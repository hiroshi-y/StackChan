// power_lite — 軽量電源管理 (cat-box の cat_power 相当を mooncake 版へ移植)。
//
// ※ 現状 esp_pm(DFS / 自動ライトスリープ)は無効(CONFIG_PM_ENABLE 未設定)。
//   このファーム(LVGL+PSRAM フレームバッファ+アバター+カメラ)では DFS/light sleep が
//   PSRAM/表示まわりのメモリ破壊を招き、暗転後タッチ→アプリ起動時に PANIC した。
//   そのため本モジュールは inert(無効時は全 API が no-op)。省電力は #4 のバックライト
//   自動消灯(hal.cpp, 15 秒)が担う。将来 CPU 省電力を入れるなら PSRAM+PM の整合を要検討。
//
//   API は将来の再有効化に備えて残してある:
//   #2 抑止ロック: ESP_PM_NO_LIGHT_SLEEP ロックで「ライトスリープしてよい/だめ」を制御。
//      画面OFF かつ 重要モード非稼働のときだけ許可。CAT(USB)/QR(カメラ) は keep_awake で抑止。
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// esp_pm を構成し、NO_LIGHT_SLEEP ロックを生成して取得(=既定は抑止)する。
// CONFIG_PM_ENABLE が無効だと esp_pm_configure が失敗するため no-op に縮退する(安全)。
void power_lite_init(void);

// 画面(バックライト)の ON/OFF を通知。OFF かつ非 keep_awake のときだけライトスリープ許可。
void power_lite_set_screen_off(bool off);

// USB ホスト(CAT)・カメラ(QR)等、ライトスリープで壊れる処理の稼働中に true。
// true の間は画面 OFF でもライトスリープを抑止する。
void power_lite_keep_awake(bool on);

#ifdef __cplusplus
}
#endif
