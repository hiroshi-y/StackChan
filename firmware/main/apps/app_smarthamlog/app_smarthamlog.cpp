/*
 * Smart HAMLOG app (Phase 0)
 * onOpen で USB ホストを起動(VBUS + install)し、無線機(CP210x)の検出・周波数を画面表示する。
 */
#include "app_smarthamlog.h"
#include "cat_core.hpp"
#include "cat_bridge.hpp"
#include <hal/hal.h>
#include <mooncake.h>
#include <mooncake_log.h>
#include <smooth_lvgl.hpp>
#include <assets/assets.h>            // assets::get_image(アイコン)
#include "esp_system.h"               // esp_reset_reason / esp_get_free_heap_size
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"            // xTaskCreate(QUIT の非同期停止)

using namespace mooncake;
using namespace smooth_ui_toolkit::lvgl_cpp;

AppSmartHamlog::AppSmartHamlog()
{
    setAppInfo().name = "Smart HAMLOG";
    // アイコン(暫定: 既存のコントローラアイコンを流用。専用ラジオアイコンは後日アセット生成)
    static auto icon = assets::get_image("icon_controller.bin");
    setAppInfo().icon = (void*)&icon;
}

static std::unique_ptr<Button> _btn_quit;
static lv_obj_t* _lbl_title  = nullptr;
static lv_obj_t* _lbl_status = nullptr;
static lv_obj_t* _lbl_freq   = nullptr;
static lv_obj_t* _lbl_log    = nullptr;
static uint32_t  _tick       = 0;
static bool      _started    = false;
static bool      _need_start = false;
static const char* _rst_name = "?";   // 前回のリセット要因(リブート診断用)

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

static const char* wifi_status_str(WifiStatus s)
{
    switch (s) {
        case WifiStatus::None:   return "OFF";
        case WifiStatus::Low:    return "Low";
        case WifiStatus::Medium: return "Mid";
        case WifiStatus::High:   return "High";
        default:                 return "?";
    }
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
    LvglLockGuard lock;

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

    // ステータス(上部): WiFi + RIG 接続
    _lbl_status = lv_label_create(lv_screen_active());
    lv_label_set_text(_lbl_status, "WiFi:--  RIG:--");
    lv_obj_set_style_text_color(_lbl_status, lv_color_white(), 0);
    lv_obj_set_style_text_font(_lbl_status, &lv_font_montserrat_20, 0);
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
    _btn_quit->setAlign(LV_ALIGN_BOTTOM_MID);
    _btn_quit->label().setText("QUIT");
    _btn_quit->onClick().connect([this]() { close(); });

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
        cat_core_start();
        cat_bridge_start();   // WiFi 確保 → Worker(WSS)接続
        return;
    }

    if (GetHAL().millis() - _tick < 500) return;
    _tick = GetHAL().millis();
    if (!_lbl_status) return;

    LvglLockGuard lock;
    char sb[64];
    snprintf(sb, sizeof(sb), "WiFi:%s WS:%s RIG:%s",
             wifi_status_str(GetHAL().getWifiStatus()),
             cat_bridge_ws_connected() ? "OK" : "--",
             cat_core_connected() ? "OK" : "--");
    lv_label_set_text(_lbl_status, sb);

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

    lv_label_set_text(_lbl_log, cat_core_last_log());
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

    if (_started) {
        _started = false;   // 次回入室で再度起動できるように
        xTaskCreate(shl_teardown_task, "shl_stop", 4096, nullptr, 5, nullptr);
    }

    LvglLockGuard lock;
    _btn_quit.reset();
    if (_lbl_title)  { lv_obj_del(_lbl_title);  _lbl_title  = nullptr; }
    if (_lbl_status) { lv_obj_del(_lbl_status); _lbl_status = nullptr; }
    if (_lbl_freq)   { lv_obj_del(_lbl_freq);   _lbl_freq   = nullptr; }
    if (_lbl_log)    { lv_obj_del(_lbl_log);    _lbl_log    = nullptr; }
    // 注: Phase 0 では USB ホストは起動したまま(StackChan は USB-OTG 未使用なので無害)。
    //     書き込みし直すときは電源再投入でメニューに戻れば JTAG が復活する。
}
