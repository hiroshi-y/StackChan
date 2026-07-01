/*
 * Smart HAMLOG app (Phase 0)
 * onOpen で USB ホストを起動(VBUS + install)し、無線機(CP210x)の検出・周波数を画面表示する。
 */
#include "app_smarthamlog.h"
#include "cat_core.hpp"
#include "cat_bridge.hpp"
#include "cat_qr.hpp"
#include "cat_provision.hpp"
#include <hal/hal.h>
#include <mooncake.h>
#include <mooncake_log.h>
#include <smooth_lvgl.hpp>
#include <assets/assets.h>            // assets::get_image(アイコン)/ OGG_NEW_NOTIFICATION
#include "hal/board/hal_bridge.h"     // app_play_sound(スキャン完了ビープ)
#include "esp_system.h"               // esp_reset_reason / esp_get_free_heap_size
#include "esp_wifi.h"                 // esp_wifi_sta_get_ap_info(実接続 SSID 取得)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"            // xTaskCreate(QUIT の非同期停止)
#include "esp_heap_caps.h"           // プレビュー RGB565 バッファ(PSRAM)
#include <cstring>                    // memcpy
#include "hal/hal.h"                  // GetHAL().setBacklightAutoOff(15秒バックライト)

using namespace mooncake;
using namespace smooth_ui_toolkit::lvgl_cpp;

AppSmartHamlog::AppSmartHamlog()
{
    setAppInfo().name = "Smart HAMLOG";
    // アイコン: Smart HAMLOG 公式アイコン (icon-256.png を 150x150 RGB565A8 へ変換)。
    static auto icon = assets::get_image("icon_smarthamlog.bin");
    setAppInfo().icon = (void*)&icon;
}

static std::unique_ptr<Button> _btn_quit;
static lv_obj_t* _bg         = nullptr;   // 暗色の全画面背景(白文字を確実に見せるため)
static lv_obj_t* _lbl_title  = nullptr;
static lv_obj_t* _lbl_status = nullptr;
static lv_obj_t* _lbl_batt   = nullptr;   // 電池残量(左上)
static lv_obj_t* _lbl_freq   = nullptr;
static lv_obj_t* _lbl_log    = nullptr;
static std::unique_ptr<Button> _btn_pair;
static uint32_t  _tick       = 0;
static bool      _started    = false;
static bool      _need_start = false;
static const char* _rst_name = "?";   // 前回のリセット要因(リブート診断用)

// ---- QR ペアリング ----
static bool          _pairing      = false;
static volatile bool _want_pair    = false;   // PAIR 押下(onRunning で cat_core 停止→カメラ起動)
static volatile bool _pair_pending = false;   // デコード成功(callback で立つ)
static char          _pair_payload[512];
static int           _pair_len     = 0;
static char          _pair_msg[224] = "";
static uint32_t      _led_off_at   = 0;         // 完了 LED を消す時刻(millis、0=点灯予定なし)
static lv_obj_t*     _img_preview  = nullptr;   // カメラプレビュー canvas(ペアリング中のみ表示)
static uint16_t*     _preview_rgb  = nullptr;   // canvas の RGB565 バッファ(PSRAM)

static void on_qr_decoded(const char *payload, int len)
{
    if (len > (int)sizeof(_pair_payload) - 1) len = sizeof(_pair_payload) - 1;
    memcpy(_pair_payload, payload, len);
    _pair_payload[len] = '\0';
    _pair_len = len;
    _pair_pending = true;
    // 注: 音/NVS/画面処理は scan_task(PSRAM スタック)からは行わない。app スレッドで処理する。
}

static const char* reset_reason_name(esp_reset_reason_t r)
{
    switch (r) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_SW:        return "SW";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        default:                return "OTHER";
    }
}

// 実際に接続中の SSID を返す(QR で渡した SSID と、StackChan が以前から記憶していた別の
// SSID のどちらに繋がったか分かるように)。未接続は "OFF"。
static const char* wifi_ssid_str(void)
{
    static char buf[40];
    if (GetHAL().getWifiStatus() != WifiStatus::None) {
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK && ap.ssid[0]) {
            snprintf(buf, sizeof(buf), "%s", (const char*)ap.ssid);
            return buf;
        }
        return "ON";
    }
    return "OFF";
}

void AppSmartHamlog::onCreate()
{
    mclog::tagInfo(getAppInfo().name, "on create");
    _rst_name = reset_reason_name(esp_reset_reason());   // 前回リセット要因を記録
    cat_core_init();   // ドライバ登録のみ(USB ホストはまだ起動しない)
}

void AppSmartHamlog::onOpen()
{
    mclog::tagInfo(getAppInfo().name, "on open");
    // Smart HAMLOG 在室中だけ「15 秒バックライト消灯のみ」の電源管理にする:
    //   - 15 秒バックライト自動消灯を有効化(タップ復帰の握りつぶし付き)
    //   - 標準の省電力(PowerSaveTimer: 画面OFF/シャットダウン)を抑止(CAT 中の自動シャットダウン防止)
    // 退室(onClose)で両方とも既定へ戻す。スクリーンセーバーはランチャー専用なので在室中は元々動かない。
    GetHAL().setBacklightAutoOff(true);
    hal_bridge::board_set_power_save_suppressed(true);
    LvglLockGuard lock;

    // 暗色の全画面背景を必ず敷く。これを作らないと、ランチャーの動的背景色(smarthamlog は
    // 既定 0xDADADA=薄いグレー)を継承したままとなり、背景なしの白文字ラベル(ステータス/
    // 診断/ログ)が薄い背景に埋もれて見えなくなる。最初に作るので最背面。
    _bg = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(_bg);
    lv_obj_set_size(_bg, LV_PCT(100), LV_PCT(100));
    lv_obj_align(_bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(_bg, lv_color_hex(0x0b1f2a), 0);
    lv_obj_set_style_bg_opa(_bg, LV_OPA_COVER, 0);
    lv_obj_clear_flag(_bg, LV_OBJ_FLAG_SCROLLABLE);

    // カメラプレビュー(背景・初期は非表示。ペアリング中のみ表示)。最初に作るので最背面。
    if (!_preview_rgb)
        _preview_rgb = (uint16_t*)heap_caps_malloc(cat_qr_width() * cat_qr_height() * 2, MALLOC_CAP_SPIRAM);
    _img_preview = lv_canvas_create(lv_screen_active());   // cat-box と同じく canvas + RGB565
    lv_obj_align(_img_preview, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_add_flag(_img_preview, LV_OBJ_FLAG_HIDDEN);
    if (_preview_rgb)
        lv_canvas_set_buffer(_img_preview, _preview_rgb, cat_qr_width(), cat_qr_height(), LV_COLOR_FORMAT_RGB565);

    // 題字: 全幅のティール色ヘッダ + 白の大きい文字で目立たせる
    _lbl_title = lv_label_create(lv_screen_active());
    lv_label_set_text(_lbl_title, "Smart HAMLOG");
    lv_obj_set_style_text_color(_lbl_title, lv_color_white(), 0);
    lv_obj_set_style_text_font(_lbl_title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_align(_lbl_title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(_lbl_title, LV_PCT(100));
    lv_obj_set_style_bg_color(_lbl_title, lv_color_hex(0x0E7490), 0);  // teal
    lv_obj_set_style_bg_opa(_lbl_title, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_top(_lbl_title, 8, 0);
    lv_obj_set_style_pad_bottom(_lbl_title, 8, 0);
    lv_obj_align(_lbl_title, LV_ALIGN_TOP_MID, 0, 0);

    // 電池残量(左上・題字ヘッダの上に重ねる)。題字の後に作って前面に。
    _lbl_batt = lv_label_create(lv_screen_active());
    lv_label_set_text(_lbl_batt, "");
    lv_obj_set_style_text_color(_lbl_batt, lv_color_white(), 0);
    lv_obj_set_style_text_font(_lbl_batt, &lv_font_montserrat_16, 0);
    lv_obj_align(_lbl_batt, LV_ALIGN_TOP_LEFT, 6, 10);

    // ステータス(上部): WiFi + RIG 接続
    _lbl_status = lv_label_create(lv_screen_active());
    // 接続 SSID を出すと長くなる場合があるので、横幅を決めて長文はスクロールさせる。
    lv_label_set_long_mode(_lbl_status, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_width(_lbl_status, 304);
    lv_label_set_text(_lbl_status, "WiFi:--  RIG:--");
    lv_obj_set_style_text_color(_lbl_status, lv_color_white(), 0);
    lv_obj_set_style_text_font(_lbl_status, &lv_font_montserrat_16, 0);
    lv_obj_align(_lbl_status, LV_ALIGN_TOP_LEFT, 8, 48);

    // 診断(上部): 空きヒープ / 前回リセット要因 / RX 周波数
    _lbl_freq = lv_label_create(lv_screen_active());
    lv_label_set_text(_lbl_freq, "");
    lv_obj_set_style_text_color(_lbl_freq, lv_color_white(), 0);
    lv_obj_set_style_text_font(_lbl_freq, &lv_font_montserrat_16, 0);
    lv_obj_align(_lbl_freq, LV_ALIGN_TOP_LEFT, 8, 78);

    // ログ(中央)
    _lbl_log = lv_label_create(lv_screen_active());
    lv_label_set_long_mode(_lbl_log, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(_lbl_log, 300);
    lv_label_set_text(_lbl_log, "");
    lv_obj_set_style_text_color(_lbl_log, lv_color_white(), 0);
    lv_obj_set_style_text_font(_lbl_log, &lv_font_montserrat_16, 0);
    lv_obj_align(_lbl_log, LV_ALIGN_CENTER, 0, 24);

    _btn_quit = std::make_unique<Button>(lv_screen_active());
    _btn_quit->setSize(84, 44);    // 題字(24)より小さく。タップはしやすいサイズ
    _btn_quit->setAlign(LV_ALIGN_BOTTOM_LEFT);   // 左下(分かりやすさのため QUIT を左に)
    _btn_quit->label().setText("QUIT");
    _btn_quit->label().setTextFont(&lv_font_montserrat_16);
    _btn_quit->onClick().connect([this]() { close(); });

    // PAIR: カメラで QR を読んで WiFi/トークンをプロビジョニング
    _btn_pair = std::make_unique<Button>(lv_screen_active());
    _btn_pair->setSize(84, 44);
    _btn_pair->setAlign(LV_ALIGN_BOTTOM_RIGHT);   // 右下(QUIT と左右入れ替え)
    _btn_pair->label().setText("PAIR");
    _btn_pair->label().setTextFont(&lv_font_montserrat_16);
    _btn_pair->onClick().connect([this]() {
        // 実処理(cat_core 停止=~1.5s + カメラ起動)は onRunning 側で行う。
        // ここ(LVGL イベント)でブロックしないようフラグだけ立てる。
        if (!_pairing && !_want_pair) _want_pair = true;
    });

    // USB ホスト起動(usb_host_install 等)は重い。LVGL ロックを握った onOpen 内で
    // 実行すると UI が描画される前にブロック/パニックし得るので、UI を先に出してから
    // 初回 onRunning で起動する(下記 onRunning の _need_start 分岐)。
    if (!_started) _need_start = true;
}

void AppSmartHamlog::onRunning()
{
    // 初回: UI 描画後に USB ホストを起動(onOpen から遅延)。LVGL ロックは握らない。
    if (_need_start) {
        _need_start = false;
        _started = true;
        // このモードに入ったら、ブラウザの操作を待たずに自動でリグ(USB ホスト)に接続する。
        //  - cat_bridge: WiFi 確保 → Worker(WSS)接続
        //  - cat_core:   USB ホスト起動 → リグを列挙(baud は既定 19200。ブラウザの "open" が
        //                来れば baud を上書き)。
        // 注意: USB ホスト起動で COM9(USB-Serial/JTAG)は切れる。書き込み時は QUIT か再起動で
        //       メニューに戻れば COM9 が復帰する。ペアリング(カメラ)時はリソース競合を避けるため
        //       cat_core を一時停止する(PAIR ハンドラ参照)。
        cat_bridge_start();
        cat_core_start();
        return;
    }

    // PAIR 押下: カメラ(QR)と USB ホストのリソース競合を避けるため、先に cat_core を止めてから
    // カメラを起動する(cat_core_stop は ~1.5s ブロックするので LVGL イベントではなくここで実行)。
    if (_want_pair && !_pairing) {
        _want_pair = false;
        cat_core_stop();            // リグを一時停止(ペアリング後に再開する)
        _pair_pending = false;
        _pair_msg[0] = '\0';
        if (cat_qr_begin(on_qr_decoded)) {
            _pairing = true;
            LvglLockGuard lock;
            if (_btn_pair) lv_obj_add_flag(_btn_pair->get(), LV_OBJ_FLAG_HIDDEN);  // QUIT だけにする
        } else {
            cat_core_start();        // カメラ起動に失敗したらリグを戻す
        }
        return;
    }

    // ペアリング中: QR スキャン状態を表示し、デコードできたらプロビジョニングを適用。
    if (_pairing) {
        if (_pair_pending) {
            _pair_pending = false;
            cat_qr_end();
            char sum[96] = {0};
            bool ok = cat_provision_apply_json(_pair_payload, _pair_len, sum, sizeof(sum));
            if (ok) {
                // 追加した WiFi(QR)と、今つながっている WiFi は別のことがある(StackChan は
                // 記憶済みの中で電波が届くものに繋ぐ)。両方を表示する。
                snprintf(_pair_msg, sizeof(_pair_msg),
                         "Paired! %s\nNow connected: %s\nQUIT & re-enter -> WS", sum, wifi_ssid_str());
                GetHAL().showRgbColor(0, 70, 0);   // 緑 LED で完了合図(音は未対応のため暫定)
            } else {
                snprintf(_pair_msg, sizeof(_pair_msg), "QR read OK but parse failed");
                GetHAL().showRgbColor(70, 0, 0);   // 赤 LED
            }
            _led_off_at = GetHAL().millis() + 5000;   // 5秒で消灯(消費電力対策)
            _pairing = false;
            {
                LvglLockGuard lock;   // 通常 UI に戻す
                if (_img_preview) lv_obj_add_flag(_img_preview, LV_OBJ_FLAG_HIDDEN);
                if (_lbl_title)   lv_obj_clear_flag(_lbl_title, LV_OBJ_FLAG_HIDDEN);
                if (_lbl_freq)    lv_obj_clear_flag(_lbl_freq, LV_OBJ_FLAG_HIDDEN);
                if (_btn_pair)    lv_obj_clear_flag(_btn_pair->get(), LV_OBJ_FLAG_HIDDEN);
            }   // ロック解放後に
            cat_core_start();   // ペアリングで止めていたリグを再開
        } else {
            if (GetHAL().millis() - _tick < 200) return;
            _tick = GetHAL().millis();
            if (!_lbl_status) return;
            LvglLockGuard lock;
            // プレビュー更新(最新グレースケールフレーム)
            const uint8_t *g = cat_qr_gray();
            if (g && _preview_rgb && _img_preview) {
                int npix = cat_qr_width() * cat_qr_height();
                for (int i = 0; i < npix; i++) {
                    uint8_t v = g[i];
                    _preview_rgb[i] = (uint16_t)(((v >> 3) << 11) | ((v >> 2) << 5) | (v >> 3));
                }
                lv_obj_clear_flag(_img_preview, LV_OBJ_FLAG_HIDDEN);
                lv_obj_invalidate(_img_preview);   // canvas はバッファを直接指すので invalidate のみ
            }
            // プレビューを見やすくするため題字/診断は隠す。PAIR ボタンもペアリング中は隠し
            // (QUIT だけ残す)。
            if (_lbl_title) lv_obj_add_flag(_lbl_title, LV_OBJ_FLAG_HIDDEN);
            if (_lbl_freq)  lv_obj_add_flag(_lbl_freq, LV_OBJ_FLAG_HIDDEN);
            if (_btn_pair)  lv_obj_add_flag(_btn_pair->get(), LV_OBJ_FLAG_HIDDEN);
            char sb[48];
            snprintf(sb, sizeof(sb), "Scan QR  frames:%u", (unsigned)cat_qr_frame_count());
            lv_label_set_text(_lbl_status, sb);
            lv_label_set_text(_lbl_log, cat_qr_status());
            return;
        }
    }

    if (GetHAL().millis() - _tick < 500) return;
    _tick = GetHAL().millis();

    // 完了 LED は数秒で消す(点けっぱなしは消費電力が大きい)
    if (_led_off_at && GetHAL().millis() >= _led_off_at) {
        GetHAL().showRgbColor(0, 0, 0);
        _led_off_at = 0;
    }

    if (!_lbl_status) return;

    LvglLockGuard lock;
    char sb[96];
    snprintf(sb, sizeof(sb), "WiFi:%s WS:%s RIG:%s",
             wifi_ssid_str(),
             cat_bridge_ws_connected() ? "OK" : "--",
             cat_core_connected() ? "OK" : "--");
    lv_label_set_text(_lbl_status, sb);

    if (_lbl_batt) {   // 電池アイコン + 残量%(残量でアイコンを切替、充電中は先頭に充電マーク)
        const int  lvl = (int)GetHAL().getBatteryLevel();
        const bool chg = GetHAL().isBatteryCharging();
        const char* sym = lvl >= 90 ? LV_SYMBOL_BATTERY_FULL
                        : lvl >= 70 ? LV_SYMBOL_BATTERY_3
                        : lvl >= 40 ? LV_SYMBOL_BATTERY_2
                        : lvl >= 15 ? LV_SYMBOL_BATTERY_1
                        :             LV_SYMBOL_BATTERY_EMPTY;
        char bb[24];
        // 例: "🔋 73%" / 充電中 "⚡🔋 73%"
        snprintf(bb, sizeof(bb), "%s%s %d%%", chg ? LV_SYMBOL_CHARGE : "", sym, lvl);
        lv_label_set_text(_lbl_batt, bb);
    }

    // 診断: 空きヒープ + 前回リセット要因 + RX 周波数。WS 未接続でエラーがあれば WS エラーを表示。
    char fb[80];
    unsigned heapK = (unsigned)(esp_get_free_heap_size() / 1024);
    const char* werr = cat_bridge_last_err();
    if (!cat_bridge_ws_connected() && werr[0] != '\0') {
        snprintf(fb, sizeof(fb), "h:%uK WSerr:%s", heapK, werr);
    } else {
        double f = cat_core_last_freq_mhz();
        char fstr[16];
        if (f > 0.0) snprintf(fstr, sizeof(fstr), "%.4f", f);
        else         { fstr[0] = '-'; fstr[1] = '\0'; }
        snprintf(fb, sizeof(fb), "h:%uK r:%s RX:%s", heapK, _rst_name, fstr);
    }
    lv_label_set_text(_lbl_freq, fb);

    lv_label_set_text(_lbl_log, _pair_msg[0] ? _pair_msg : cat_core_last_log());
}

// 停止処理(USB host/WSS の解放)は ~1.5s かかる。onClose で同期実行すると QUIT の
// 反応が悪くなるため、バックグラウンドタスクで行う。メニューには即戻る。
static void shl_teardown_task(void *arg)
{
    cat_bridge_stop();   // WSS 切断 + 無線機 RX タップ解除
    cat_core_stop();     // USB host 停止 → USB-OTG PHY 解放(COM9 復帰、数秒後)
    vTaskDelete(nullptr);
}

void AppSmartHamlog::onClose()
{
    mclog::tagInfo(getAppInfo().name, "on close");
    // 在室中の電源管理を既定へ戻す: 15 秒自動消灯を無効化(=点灯に戻す)+ 標準省電力の抑止を解除。
    GetHAL().setBacklightAutoOff(false);
    hal_bridge::board_set_power_save_suppressed(false);

    if (_pairing) { cat_qr_end(); _pairing = false; }

    if (_led_off_at) { GetHAL().showRgbColor(0, 0, 0); _led_off_at = 0; }   // 退室時に LED 消灯

    if (_started) {
        _started = false;   // 次回入室で再度起動できるように
        xTaskCreate(shl_teardown_task, "shl_stop", 4096, nullptr, 5, nullptr);
    }

    LvglLockGuard lock;
    _btn_quit.reset();
    _btn_pair.reset();
    if (_img_preview){ lv_obj_del(_img_preview);_img_preview= nullptr; }
    if (_bg)         { lv_obj_del(_bg);         _bg         = nullptr; }
    if (_lbl_title)  { lv_obj_del(_lbl_title);  _lbl_title  = nullptr; }
    if (_lbl_batt)   { lv_obj_del(_lbl_batt);   _lbl_batt   = nullptr; }
    if (_lbl_status) { lv_obj_del(_lbl_status); _lbl_status = nullptr; }
    if (_lbl_freq)   { lv_obj_del(_lbl_freq);   _lbl_freq   = nullptr; }
    if (_lbl_log)    { lv_obj_del(_lbl_log);    _lbl_log    = nullptr; }
    // 注: Phase 0 では USB ホストは起動したまま(StackChan は USB-OTG 未使用なので無害)。
    //     書き込みし直すときは電源再投入でメニューに戻れば JTAG が復活する。
}
