/*
 * Smart HAMLOG app (Phase 0)
 * onOpen で USB ホストを起動(VBUS + install)し、無線機(CP210x)の検出・周波数を画面表示する。
 */
#include "app_smarthamlog.h"
#include "cat_core.hpp"
#include <hal/hal.h>
#include <mooncake.h>
#include <mooncake_log.h>
#include <smooth_lvgl.hpp>

using namespace mooncake;
using namespace smooth_ui_toolkit::lvgl_cpp;

AppSmartHamlog::AppSmartHamlog()
{
    setAppInfo().name = "Smart HAMLOG";
    // setAppInfo().icon = (void*)&icon_app_dummy;  // アイコンは Phase 1 で
}

static std::unique_ptr<Button> _btn_quit;
static lv_obj_t* _lbl_title  = nullptr;
static lv_obj_t* _lbl_status = nullptr;
static lv_obj_t* _lbl_freq   = nullptr;
static lv_obj_t* _lbl_log    = nullptr;
static uint32_t  _tick       = 0;
static bool      _started    = false;
static bool      _need_start = false;

void AppSmartHamlog::onCreate()
{
    mclog::tagInfo(getAppInfo().name, "on create");
    cat_core_init();   // ドライバ登録のみ(USB ホストはまだ起動しない)
}

void AppSmartHamlog::onOpen()
{
    mclog::tagInfo(getAppInfo().name, "on open");
    LvglLockGuard lock;

    _lbl_title = lv_label_create(lv_screen_active());
    lv_label_set_text(_lbl_title, "Smart HAMLOG (CAT)");
    lv_obj_align(_lbl_title, LV_ALIGN_TOP_MID, 0, 8);

    _lbl_status = lv_label_create(lv_screen_active());
    lv_label_set_text(_lbl_status, "Starting USB host...");
    lv_obj_align(_lbl_status, LV_ALIGN_TOP_LEFT, 8, 40);

    _lbl_freq = lv_label_create(lv_screen_active());
    lv_label_set_text(_lbl_freq, "RX: --- MHz");
    lv_obj_align(_lbl_freq, LV_ALIGN_TOP_LEFT, 8, 68);

    _lbl_log = lv_label_create(lv_screen_active());
    lv_label_set_long_mode(_lbl_log, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(_lbl_log, 300);
    lv_label_set_text(_lbl_log, "");
    lv_obj_align(_lbl_log, LV_ALIGN_TOP_LEFT, 8, 96);

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
        return;
    }

    if (GetHAL().millis() - _tick < 500) return;
    _tick = GetHAL().millis();
    if (!_lbl_status) return;

    LvglLockGuard lock;
    lv_label_set_text(_lbl_status, cat_core_connected() ? "RIG CONNECTED" : "Waiting for rig...");

    char fb[40];
    double f = cat_core_last_freq_mhz();
    if (f > 0.0) snprintf(fb, sizeof(fb), "RX: %.6f MHz", f);
    else         snprintf(fb, sizeof(fb), "RX: --- MHz");
    lv_label_set_text(_lbl_freq, fb);

    lv_label_set_text(_lbl_log, cat_core_last_log());
}

void AppSmartHamlog::onClose()
{
    mclog::tagInfo(getAppInfo().name, "on close");
    LvglLockGuard lock;
    _btn_quit.reset();
    if (_lbl_title)  { lv_obj_del(_lbl_title);  _lbl_title  = nullptr; }
    if (_lbl_status) { lv_obj_del(_lbl_status); _lbl_status = nullptr; }
    if (_lbl_freq)   { lv_obj_del(_lbl_freq);   _lbl_freq   = nullptr; }
    if (_lbl_log)    { lv_obj_del(_lbl_log);    _lbl_log    = nullptr; }
    // 注: Phase 0 では USB ホストは起動したまま(StackChan は USB-OTG 未使用なので無害)。
    //     書き込みし直すときは電源再投入でメニューに戻れば JTAG が復活する。
}
