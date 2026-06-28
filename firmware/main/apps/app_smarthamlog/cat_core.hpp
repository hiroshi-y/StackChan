// cat_core — USB ホスト(CP210x VCP)+ CAT(Icom CI-V / Yaesu ASCII)
// Smart HAMLOG app 版: cat-box(standalone)から移植。m5stack BSP 依存を除去し、
// StackChan の I2C バス(hal_bridge::board_get_i2c_bus)+ AW9523 で VBUS を供給、
// USB ホストは usb_host_install / cdc_acm_host_install を自前で行う。
#pragma once
#include <stdint.h>
#include <stddef.h>   // size_t

enum cat_rig_mode_t {
    CAT_MODE_ICOM_CIV = 0,
    CAT_MODE_YAESU_ASCII = 1,
    CAT_MODE_COUNT
};

// ドライバ登録 + 内部状態の初期化 (USB ホストはまだ起動しない)
void cat_core_init(void);

// USB ホストを起動 (VBUS 供給開始)。Smart HAMLOG 画面に入ったときに 1 回だけ呼ぶ。
void cat_core_start(void);

// 操作
void cat_core_set_freq_mhz(double mhz);   // 周波数設定を送信
void cat_core_read_freq(void);            // 周波数読み取りを送信

// リグ/ボーレート切替
const char *cat_core_cycle_mode(void);    // 次モードへ → 名前を返す
const char *cat_core_mode_name(void);
uint32_t    cat_core_cycle_baud(void);    // 次 baud へ → 値を返す(接続中は即適用)
uint32_t    cat_core_baud(void);
bool        cat_core_connected(void);

// ---- Phase 0 状態表示用(mooncake UI から参照)----
double       cat_core_last_freq_mhz(void); // 直近に解析した RX 周波数(MHz)。未取得は 0
const char  *cat_core_last_log(void);      // 直近のログ 1 行(状態表示用)

// ---- ブリッジ(ダムパイプ)用: 生バイトのパススルー ----
typedef void (*cat_raw_rx_cb)(const uint8_t *data, size_t len);
void cat_core_set_raw_rx(cat_raw_rx_cb cb);   // nullptr で解除
void cat_core_write_raw(const uint8_t *data, size_t len);  // 無線機へ生バイト送信
void cat_core_set_baud(uint32_t baud);        // 任意 baud を即適用(接続中)/次回 open にも反映
