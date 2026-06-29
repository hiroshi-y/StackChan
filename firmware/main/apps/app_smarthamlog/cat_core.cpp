// cat_core — USB Host (CP210x VCP) + CAT(Icom CI-V / Yaesu ASCII)
// Smart HAMLOG app 版。cat-box から移植。BSP 依存(bsp_usb_host_start / bsp_io_expander_init /
// bsp_i2c_get_handle)を除去し、StackChan の I2C バス + AW9523 で VBUS を供給、USB ホストは
// usb_host_install / cdc_acm_host_install を自前で行う。
#include "cat_core.hpp"
#include "hal/board/hal_bridge.h"   // hal_bridge::board_get_i2c_bus()
#include "driver/i2c_master.h"
#include "usb/usb_host.h"
#include "soc/usb_dwc_struct.h"
#include "usb/vcp.hpp"
#include "usb/vcp_cp210x.hpp"
#include "usb/cdc_acm_host.h"
#include "esp_private/usb_phy.h"     // usb_new_phy/usb_del_phy(停止時に PHY を JTAG へ戻す)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>

using namespace esp_usb;
static const char *TAG = "cat_core";

// ---- 状態 ----
static cat_rig_mode_t s_mode = CAT_MODE_ICOM_CIV;
static uint8_t s_civ_addr = 0x88;                 // IC-7100 既定
static const uint32_t BAUDS[] = {4800, 9600, 19200, 38400, 115200};
static const int N_BAUD = sizeof(BAUDS) / sizeof(BAUDS[0]);
static int s_baud_idx = 2;                         // 19200
static volatile bool s_connected = false;
static volatile bool s_dev_lost = false;
static bool s_started = false;
static volatile bool s_stop = false;            // 停止要求(QUIT 時に立てる)
static volatile bool s_stopping = false;        // 停止処理 実行中(再入室の競合回避)
static CdcAcmDevice *s_dev = nullptr;
static SemaphoreHandle_t s_mtx = nullptr;
static SemaphoreHandle_t s_usb_done = nullptr;   // usb_task 終了通知
static SemaphoreHandle_t s_lib_done = nullptr;   // usb_lib_task 終了通知
static usb_phy_handle_t  s_jtag_phy = nullptr;   // 停止時に作る USB-Serial/JTAG PHY(COM9 復帰)
static cat_raw_rx_cb s_raw_sink = nullptr;   // ブリッジ用: 生 RX タップ
static uint32_t s_baud_override = 0;          // 0=BAUDS テーブル / 非0=任意 baud

// ---- UI 状態(cat_ui の代替: mooncake UI から参照)----
static char   s_log[80] = "";
static double s_freq_mhz = 0.0;
static void ui_log(const char *pfx, const char *msg)
{
    snprintf(s_log, sizeof(s_log), "%s %s", pfx, msg);
    ESP_LOGI(TAG, "%s %s", pfx, msg);
}
double cat_core_last_freq_mhz(void) { return s_freq_mhz; }
const char *cat_core_last_log(void) { return s_log; }

static uint32_t cur_baud(void) { return s_baud_override ? s_baud_override : BAUDS[s_baud_idx]; }

// ---- ログヘルパ ----
static void log_bytes(const char *pfx, const uint8_t *d, size_t n)
{
    char buf[200];
    size_t o = 0;
    for (size_t i = 0; i < n && o + 3 < sizeof(buf); i++) {
        o += snprintf(buf + o, sizeof(buf) - o, "%02X ", d[i]);
    }
    ui_log(pfx, buf);
}

// ---- 受信バッファ & 周波数解析 ----
static uint8_t s_rx[512];
static size_t s_rxlen = 0;
static void rx_reset(void) { s_rxlen = 0; }

static void parse_rx(void)
{
    if (s_mode == CAT_MODE_ICOM_CIV) {
        long hz = -1;
        size_t i = 0;
        while (i + 5 < s_rxlen) {
            if (s_rx[i] == 0xFE && s_rx[i + 1] == 0xFE) {
                size_t j = i + 2;
                while (j < s_rxlen && s_rx[j] != 0xFD) j++;
                if (j >= s_rxlen) break;            // 未完フレーム → 続きを待つ
                uint8_t cmd = (j - i >= 4) ? s_rx[i + 4] : 0xFF;
                size_t dstart = i + 5, dcount = (j > dstart) ? (j - dstart) : 0;
                if ((cmd == 0x03 || cmd == 0x00) && dcount == 5) {
                    char s[11];
                    int p = 0;
                    for (int k = 4; k >= 0; k--) {
                        uint8_t b = s_rx[dstart + k];
                        s[p++] = '0' + ((b >> 4) & 0xf);
                        s[p++] = '0' + (b & 0xf);
                    }
                    s[p] = 0;
                    hz = atol(s);
                }
                i = j + 1;
                continue;
            }
            i++;
        }
        if (hz >= 0) { s_freq_mhz = (double)hz / 1e6; rx_reset(); }
    } else { // Yaesu/Kenwood ASCII: FA<digits>;
        for (size_t i = 0; i + 3 < s_rxlen; i++) {
            if (s_rx[i] == 'F' && s_rx[i + 1] == 'A') {
                size_t j = i + 2;
                while (j < s_rxlen && s_rx[j] != ';') j++;
                if (j < s_rxlen) {
                    char s[16];
                    size_t p = 0;
                    for (size_t k = i + 2; k < j && p < sizeof(s) - 1; k++) {
                        if (s_rx[k] >= '0' && s_rx[k] <= '9') s[p++] = s_rx[k];
                    }
                    s[p] = 0;
                    if (p >= 8) { s_freq_mhz = atol(s) / 1e6; rx_reset(); return; }
                }
            }
        }
    }
}

// ---- USB コールバック ----
static bool data_cb(const uint8_t *data, size_t len, void *arg)
{
    // ブリッジ用タップ: 生バイトをそのまま上位(WSS)へ。周波数解析も併走。
    if (s_raw_sink) s_raw_sink(data, len);
    log_bytes("RX:", data, len);
    for (size_t i = 0; i < len; i++) {
        if (s_rxlen >= sizeof(s_rx)) s_rxlen = 0; // オーバーフロー保護
        s_rx[s_rxlen++] = data[i];
    }
    parse_rx();
    return true;
}

static void event_cb(const cdc_acm_host_dev_event_data_t *e, void *arg)
{
    if (e->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
        s_dev_lost = true;
    }
}

// ---- 送信 ----
static void send_bytes(const uint8_t *d, size_t n)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    CdcAcmDevice *dev = s_dev;
    if (dev) {
        uint8_t tmp[64];
        if (n > sizeof(tmp)) n = sizeof(tmp);
        memcpy(tmp, d, n);
        esp_err_t err = dev->tx_blocking(tmp, n, 200);
        log_bytes("TX:", d, n);
        if (err != ESP_OK) ui_log("!!", "TX error");
    } else {
        ui_log("--", "USB not connected");
    }
    xSemaphoreGive(s_mtx);
}

// ---- プロトコル組み立て ----
static void bcd_le(uint32_t hz, uint8_t out[5])
{
    char s[11];
    snprintf(s, sizeof(s), "%010lu", (unsigned long)hz); // 10 桁
    for (int k = 0; k < 5; k++) {
        int hi = s[8 - 2 * k] - '0';
        int lo = s[9 - 2 * k] - '0';
        out[k] = (uint8_t)((hi << 4) | lo);
    }
}

void cat_core_set_freq_mhz(double mhz)
{
    uint32_t hz = (uint32_t)(mhz * 1e6 + 0.5);
    if (s_mode == CAT_MODE_ICOM_CIV) {
        uint8_t b[5];
        bcd_le(hz, b);
        uint8_t f[] = {0xFE, 0xFE, s_civ_addr, 0xE0, 0x05, b[0], b[1], b[2], b[3], b[4], 0xFD};
        send_bytes(f, sizeof(f));
    } else {
        char s[16];
        snprintf(s, sizeof(s), "FA%09lu;", (unsigned long)hz);
        send_bytes((const uint8_t *)s, strlen(s));
    }
}

void cat_core_read_freq(void)
{
    if (s_mode == CAT_MODE_ICOM_CIV) {
        uint8_t f[] = {0xFE, 0xFE, s_civ_addr, 0xE0, 0x03, 0xFD};
        send_bytes(f, sizeof(f));
    } else {
        const char *s = "FA;";
        send_bytes((const uint8_t *)s, 3);
    }
}

// ---- リグ/ボーレート ----
const char *cat_core_mode_name(void)
{
    return (s_mode == CAT_MODE_ICOM_CIV) ? "Icom CI-V" : "Yaesu ASCII";
}
const char *cat_core_cycle_mode(void)
{
    s_mode = (cat_rig_mode_t)((s_mode + 1) % CAT_MODE_COUNT);
    rx_reset();
    return cat_core_mode_name();
}
uint32_t cat_core_baud(void) { return cur_baud(); }
uint32_t cat_core_cycle_baud(void)
{
    s_baud_override = 0;   // 手動切替はテーブルに戻す
    s_baud_idx = (s_baud_idx + 1) % N_BAUD;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    if (s_dev) {
        cdc_acm_line_coding_t lc;
        lc.dwDTERate = cur_baud();
        lc.bCharFormat = 0;   // 1 stop bit
        lc.bParityType = 0;   // none
        lc.bDataBits = 8;
        s_dev->line_coding_set(&lc);
    }
    xSemaphoreGive(s_mtx);
    return cur_baud();
}
bool cat_core_connected(void) { return s_connected; }

// ---- ブリッジ(ダムパイプ)用 ----
void cat_core_set_raw_rx(cat_raw_rx_cb cb) { s_raw_sink = cb; }
void cat_core_write_raw(const uint8_t *d, size_t n) { send_bytes(d, n); }
void cat_core_set_baud(uint32_t baud)
{
    s_baud_override = baud;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    if (s_dev) {
        cdc_acm_line_coding_t lc;
        lc.dwDTERate = baud;
        lc.bCharFormat = 0;   // 1 stop
        lc.bParityType = 0;   // none
        lc.bDataBits = 8;
        s_dev->line_coding_set(&lc);
    }
    xSemaphoreGive(s_mtx);
}

// ---- USB ホストライブラリのイベント処理タスク(BSP の代替)----
static void usb_lib_task(void *arg)
{
    while (true) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        // 停止時: 全デバイス解放済み(ALL_FREE)になったらループを抜ける。
        if (s_stop && (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)) {
            break;
        }
    }
    if (s_lib_done) xSemaphoreGive(s_lib_done);
    vTaskDelete(nullptr);
}

// 診断: 列挙済みデバイスの VID/PID を読み、画面ログへ(CP210x か別チップかの判定用)。
// 専用の USB クライアントを一時登録 → 各デバイスを open して device descriptor を取得。
static void diag_log_vidpid(void)
{
    usb_host_client_handle_t client = nullptr;
    usb_host_client_config_t ccfg = {};
    ccfg.is_synchronous = false;
    ccfg.max_num_event_msg = 5;
    ccfg.async.client_event_callback = [](const usb_host_client_event_msg_t *, void *) {};
    ccfg.async.callback_arg = nullptr;
    if (usb_host_client_register(&ccfg, &client) != ESP_OK || !client) return;

    uint8_t addrs[8];
    int n = 0;
    usb_host_device_addr_list_fill(8, addrs, &n);
    for (int i = 0; i < n; i++) {
        usb_device_handle_t h = nullptr;
        if (usb_host_device_open(client, addrs[i], &h) == ESP_OK && h) {
            const usb_device_desc_t *d = nullptr;
            if (usb_host_get_device_descriptor(h, &d) == ESP_OK && d) {
                char m[56];
                snprintf(m, sizeof(m), "VID=%04X PID=%04X class=%02X",
                         (unsigned)d->idVendor, (unsigned)d->idProduct, (unsigned)d->bDeviceClass);
                ui_log("ID", m);
            }
            usb_host_device_close(client, h);
        }
    }
    usb_host_client_deregister(client);
}

// ---- CDC-ACM クライアントタスク (接続/再接続を管理) ----
static void usb_task(void *arg)
{
    while (!s_stop) {
        // 診断: 列挙済みデバイス数(dev=N)+ OTG コントローラ実測。
        // dev>=1 で列挙成功(CP210x が見えている)、dev=0 で conn=1 なら列挙失敗(ハブ等)。
        {
            uint8_t addrs[8];
            int ndev = 0;
            usb_host_device_addr_list_fill(8, addrs, &ndev);
            char o[80];
            snprintf(o, sizeof(o), "dev=%d vbus=%u pwr=%u conn=%u",
                     ndev,
                     (unsigned)USB_DWC.gotgctl_reg.asesvld,
                     (unsigned)USB_DWC.hprt_reg.prtpwr,
                     (unsigned)USB_DWC.hprt_reg.prtconnsts);
            ui_log("..", o);
            if (ndev > 0) diag_log_vidpid();   // 列挙済みなら VID/PID を表示
        }

        cdc_acm_host_device_config_t cfg = {};
        cfg.connection_timeout_ms = 1000;   // 短め(停止応答とリトライを速く)
        cfg.out_buffer_size = 256;
        cfg.in_buffer_size = 256;
        cfg.event_cb = event_cb;
        cfg.data_cb = data_cb;
        cfg.user_arg = nullptr;

        CdcAcmDevice *dev = VCP::open(&cfg);   // 接続待ち(タイムアウトで nullptr)
        if (!dev) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        cdc_acm_line_coding_t lc;
        lc.dwDTERate = cur_baud();
        lc.bCharFormat = 0;
        lc.bParityType = 0;
        lc.bDataBits = 8;
        dev->line_coding_set(&lc);

        xSemaphoreTake(s_mtx, portMAX_DELAY);
        s_dev = dev;
        s_connected = true;
        xSemaphoreGive(s_mtx);
        s_dev_lost = false;
        rx_reset();
        ui_log("--", "USB connected");

        while (!s_dev_lost && !s_stop) vTaskDelay(pdMS_TO_TICKS(100));

        xSemaphoreTake(s_mtx, portMAX_DELAY);
        s_connected = false;
        CdcAcmDevice *d = s_dev;
        s_dev = nullptr;
        xSemaphoreGive(s_mtx);
        ui_log("--", "USB disconnected");
        if (d) delete d;   // デバイスを閉じる
    }
    // 停止要求でループを抜けた: 終了を通知して自タスク削除
    if (s_usb_done) xSemaphoreGive(s_usb_done);
    vTaskDelete(nullptr);
}

void cat_core_init(void)
{
    s_mtx = xSemaphoreCreateMutex();
    if (!s_usb_done) s_usb_done = xSemaphoreCreateBinary();
    if (!s_lib_done) s_lib_done = xSemaphoreCreateBinary();
    VCP::register_driver<CP210x>();   // CP2102/CP2105/CP2108
}

// AW9523(I2C 0x58)で USB-OTG の VBUS を有効化。
// StackChan のボード初期化で BOOST_EN(P1_7)/BUS_EN(P0_1)は既に ON。
// ここでは USB_OTG_EN(P0_5, reg0x02 bit5)を立てるだけで USB-C コネクタへ 5V が出る。
static void enable_usb_host_vbus(void)
{
    i2c_master_bus_handle_t bus = hal_bridge::board_get_i2c_bus();
    if (!bus) { ui_log("!!", "i2c bus unavailable"); return; }
    i2c_master_dev_handle_t aw = nullptr;
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address = 0x58;             // AW9523
    dc.scl_speed_hz = 100000;
    if (i2c_master_bus_add_device(bus, &dc, &aw) != ESP_OK || !aw) {
        ui_log("!!", "AW9523 add failed");
        return;
    }
    // P0 出力レジスタ(0x02)を read-modify-write して bit5(USB_OTG_EN)を立てる。
    uint8_t reg = 0x02, cur = 0x07;       // 読めない場合は init 既定値 0b00000111
    i2c_master_transmit_receive(aw, &reg, 1, &cur, 1, 100);
    uint8_t wr[2] = {0x02, (uint8_t)(cur | 0x20)};
    esp_err_t e = i2c_master_transmit(aw, wr, sizeof(wr), 100);
    char m[48];
    snprintf(m, sizeof(m), "USB_OTG_EN on P0=0x%02X->0x%02X", cur, wr[1]);
    ui_log(e == ESP_OK ? "--" : "!!", m);
    i2c_master_bus_rm_device(aw);
}

// AW9523 の USB_OTG_EN(P0_5)を落とす(VBUS 供給停止)。停止時に呼ぶ。
static void disable_usb_host_vbus(void)
{
    i2c_master_bus_handle_t bus = hal_bridge::board_get_i2c_bus();
    if (!bus) return;
    i2c_master_dev_handle_t aw = nullptr;
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address = 0x58;
    dc.scl_speed_hz = 100000;
    if (i2c_master_bus_add_device(bus, &dc, &aw) != ESP_OK || !aw) return;
    uint8_t reg = 0x02, cur = 0x27;
    i2c_master_transmit_receive(aw, &reg, 1, &cur, 1, 100);
    uint8_t wr[2] = {0x02, (uint8_t)(cur & ~0x20)};   // bit5 USB_OTG_EN を 0
    i2c_master_transmit(aw, wr, sizeof(wr), 100);
    i2c_master_bus_rm_device(aw);
}

void cat_core_start(void)
{
    while (s_stopping) vTaskDelay(pdMS_TO_TICKS(50));   // 前回の停止処理の完了を待つ
    if (s_started) return;
    s_started = true;
    s_stop = false;
    s_dev_lost = false;
    // 旧 stop の取りこぼし give をドレイン
    if (s_usb_done) xSemaphoreTake(s_usb_done, 0);
    if (s_lib_done) xSemaphoreTake(s_lib_done, 0);

    // 1. VBUS 供給(USB_OTG_EN を立てる)
    ui_log("--", "Enabling VBUS (USB_OTG_EN)");
    enable_usb_host_vbus();

    // 旧停止で USB-Serial/JTAG 用 PHY を作っていたら削除(OTG が PHY を取れるように)
    if (s_jtag_phy) { usb_del_phy(s_jtag_phy); s_jtag_phy = nullptr; }

    // 2. USB ホストライブラリ + CDC-ACM ドライバを自前で install(BSP の代替)
    ui_log("--", "Installing USB host");
    usb_host_config_t host_config = {};
    host_config.skip_phy_setup = false;
    host_config.intr_flags = ESP_INTR_FLAG_LEVEL1;
    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        ui_log("!!", "usb_host_install failed");
        ESP_LOGE(TAG, "usb_host_install: %d", err);
        return;
    }
    xTaskCreate(usb_lib_task, "cat_usblib", 4096, nullptr, 10, nullptr);

    err = cdc_acm_host_install(nullptr);
    if (err != ESP_OK) {
        ui_log("!!", "cdc_acm_host_install failed");
        ESP_LOGE(TAG, "cdc_acm_host_install: %d", err);
        return;
    }

    // 3. CDC-ACM クライアント(接続待ち + 中継)
    ui_log("--", "USB host started, waiting for rig");
    xTaskCreate(usb_task, "cat_usb", 4096, nullptr, 5, nullptr);
}

// USB ホストを停止して USB-OTG PHY を解放する(= USB-Serial/JTAG = COM9 が復帰)。
// QUIT 時に呼ぶ。数百ms〜1.5s ブロックする。
void cat_core_stop(void)
{
    if (!s_started) return;
    s_stopping = true;
    ui_log("--", "Stopping USB host...");
    s_stop = true;

    // 1. CDC クライアントタスク(usb_task)が device を閉じて抜けるのを待つ
    //    (VCP::open タイムアウト 1s 分の余裕をみて最大 5s)
    if (s_usb_done) xSemaphoreTake(s_usb_done, pdMS_TO_TICKS(5000));

    // 2. CDC-ACM ドライバを uninstall(クライアント解除 → lib task に NO_CLIENTS/ALL_FREE)
    cdc_acm_host_uninstall();

    // 3. usb_lib_task が ALL_FREE で抜けるのを待つ
    if (s_lib_done) xSemaphoreTake(s_lib_done, pdMS_TO_TICKS(3000));

    // 4. USB ホストライブラリを uninstall → USB-OTG PHY を解放
    usb_host_uninstall();

    // 4b. 内部 PHY を USB-Serial/JTAG に戻す(= COM9 復帰)。uninstall だけでは PHY が
    //     JTAG にルーティングされないため、明示的に SERIAL_JTAG 用 PHY を作る。
    usb_phy_config_t jc = {};
    jc.controller = USB_PHY_CTRL_SERIAL_JTAG;
    jc.target = USB_PHY_TARGET_INT;
    if (usb_new_phy(&jc, &s_jtag_phy) != ESP_OK) s_jtag_phy = nullptr;

    // 5. VBUS(USB_OTG_EN)を落とす
    disable_usb_host_vbus();

    s_connected = false;
    s_started = false;
    s_stop = false;
    s_dev = nullptr;
    s_stopping = false;
}
