// cat_hal — cat_* 共有ロジック(CAT / QR / Bridge)が使うボード依存の抽象。
//
// cat_core / cat_bridge / cat_qr / cat_provision / cat_store は StackChan と cat-box で
// バイト一致の共有コードとし、機種ごとに異なる部分だけをこの薄い HAL に閉じ込める。
// 各機種は cat_hal.cpp でこれらを実装する:
//   - StackChan: hal_bridge / GetHAL()(mooncake HAL)を包む。
//   - cat-box:   M5 BSP / cat_ui を包む。
//
// ※ 本ヘッダと上記 cat_* は「共有ソース」。変更は StackChan 側を正とし、tools の同期スクリプトで
//    cat-box へ伝播させる(コピー同期)。cat_hal.cpp(実装)だけは各リポ固有。
#pragma once

#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// UI へ 1 行ログを push する。
//   - cat-box:   ログ欄へ即時表示(push)。
//   - StackChan: 画面表示は s_log を polling して行うため no-op でよい(ESP_LOG は cat_core 側で出力済)。
void cat_hal_log(const char* pfx, const char* msg);

// AW9523 / AXP2101 を叩くための共有 I2C バス。cat_core の USB-OTG VBUS 制御に使う。
i2c_master_bus_handle_t cat_hal_i2c_bus(void);

// WiFi。cat_bridge が WSS 接続前に使う。
bool cat_hal_wifi_connected(void);   // 接続済みか
void cat_hal_wifi_start(void);       // 未接続なら接続を開始(非同期でよい)

// WiFi 認証情報を保存(cat_provision が QR プロビジョニングで使う)。複数呼ばれ得る。
void cat_hal_wifi_add_ssid(const char* ssid, const char* pass);

#ifdef __cplusplus
}
#endif
