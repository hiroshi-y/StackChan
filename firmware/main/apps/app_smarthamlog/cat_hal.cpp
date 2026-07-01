// cat_hal(StackChan 実装)。cat_hal.h の抽象を StackChan の HAL(hal_bridge / GetHAL)へ橋渡しする。
// ※ このファイルは StackChan 固有。共有対象は cat_hal.h と cat_core/cat_bridge/cat_qr/cat_provision/cat_store。
#include "cat_hal.h"
#include "hal/board/hal_bridge.h"   // hal_bridge::board_get_i2c_bus()
#include "hal/hal.h"                // GetHAL(), WifiStatus
#include "ssid_manager.h"           // SsidManager(WiFi 認証情報の保存)
#include "esp_log.h"
#include <string>
#include <string_view>

// StackChan はログを cat_core の s_log 経由で app_smarthamlog が polling 表示するため、
// ここでの push は不要(ESP_LOG は cat_core 側で出力済)。
void cat_hal_log(const char* pfx, const char* msg)
{
    (void)pfx;
    (void)msg;
}

i2c_master_bus_handle_t cat_hal_i2c_bus(void)
{
    return hal_bridge::board_get_i2c_bus();
}

bool cat_hal_wifi_connected(void)
{
    return GetHAL().getWifiStatus() != WifiStatus::None;
}

void cat_hal_wifi_start(void)
{
    if (GetHAL().getWifiStatus() == WifiStatus::None) {
        GetHAL().startNetwork([](std::string_view m) {
            ESP_LOGI("cat_hal", "net: %.*s", (int)m.size(), m.data());
        });
    }
}

void cat_hal_wifi_add_ssid(const char* ssid, const char* pass)
{
    SsidManager::GetInstance().AddSsid(ssid, pass);
}
