// power_lite — esp_pm 自動ライトスリープ + NO_LIGHT_SLEEP ロック管理。
// cat-box (firmware/cat-box/main/cat_power.cpp) の設計を mooncake 版へ移植したもの。
#include "power_lite.h"
#include "esp_pm.h"
#include "esp_log.h"

static const char* TAG = "power_lite";

static esp_pm_lock_handle_t s_no_ls = nullptr;
static bool s_lock_held   = false;   // NO_LIGHT_SLEEP ロック保持中(= ライトスリープ抑止中)か
static bool s_screen_off  = false;   // バックライト消灯中か(#4 から通知)
static bool s_keep_awake  = false;   // CAT/カメラ等の重要モード稼働中か

// ライトスリープを許可してよいのは「画面OFF かつ 重要モード非稼働」のときだけ。
// それ以外はロックを保持して抑止する。状態変化時のみ acquire/release する。
static void apply(void)
{
    if (!s_no_ls) return;  // PM 無効 or ロック生成失敗時は何もしない
    const bool want_hold = !(s_screen_off && !s_keep_awake);
    if (want_hold == s_lock_held) return;
    if (want_hold) {
        esp_pm_lock_acquire(s_no_ls);
        s_lock_held = true;
    } else {
        esp_pm_lock_release(s_no_ls);
        s_lock_held = false;
    }
    ESP_LOGD(TAG, "light_sleep %s (screen_off=%d keep_awake=%d)",
             s_lock_held ? "SUPPRESS" : "ALLOW", (int)s_screen_off, (int)s_keep_awake);
}

void power_lite_init(void)
{
    // esp_pm(DFS / 自動ライトスリープ)は現状無効化している(CONFIG_PM_ENABLE 未設定)。
    // 理由: このファーム(LVGL + PSRAM フレームバッファ + アバター + カメラ)では、CPU 周波数
    // スケーリングやライトスリープが PSRAM/表示まわりのメモリ破壊を招き、暗転後タッチ→アプリ
    // 起動時に LVGL テーマ適用で LoadProhibited PANIC を起こした(EXCVADDR が野良ポインタ)。
    // 省電力は #4 のバックライト 15 秒消灯が担う(LCD バックライトが最大の消費源)。
    // 将来 CPU 省電力を入れるなら、PSRAM + PM の整合(クロック源・APB ロック)を別途詰めること。
#if CONFIG_PM_ENABLE
    esp_pm_config_t pm = {};
    pm.max_freq_mhz       = 240;
    pm.min_freq_mhz       = 40;
    pm.light_sleep_enable = false;   // light sleep は PANIC を招くため無効
    esp_err_t e = esp_pm_configure(&pm);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "esp_pm_configure failed: 0x%x", (int)e);
        return;
    }
    if (esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "no_ls", &s_no_ls) == ESP_OK) {
        esp_pm_lock_acquire(s_no_ls);
        s_lock_held = true;
    }
    ESP_LOGI(TAG, "init: DFS enabled (light-sleep disabled)");
#else
    ESP_LOGI(TAG, "init: power management disabled (PANIC/corruption 回避); backlight-off で省電力");
#endif
}

void power_lite_set_screen_off(bool off)
{
    if (off == s_screen_off) return;
    s_screen_off = off;
    apply();
}

void power_lite_keep_awake(bool on)
{
    if (on == s_keep_awake) return;
    s_keep_awake = on;
    apply();
}
