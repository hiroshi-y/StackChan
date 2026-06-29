// cat_provision — QR の JSON を解釈してプロビジョニング(cat-box の cat_prov_save_json 相当)。
#include "cat_provision.hpp"
#include "cat_store.hpp"
#include "ssid_manager.h"     // StackChan(esp-wifi-connect)の WiFi 保存
#include "cJSON.h"
#include "esp_log.h"
#include <cstring>

static const char *TAG = "cat_prov";

// h(ホスト): 1=本番 / 2=staging / 文字列=URL
static const char *HOST_PROD    = "smart-hamlog-cat.hiroshi-a4b.workers.dev";
static const char *HOST_STAGING = "staging-smart-hamlog-cat.hiroshi-a4b.workers.dev";

bool cat_provision_apply_json(const char *json, int len, int *wifi_added)
{
    if (wifi_added) *wifi_added = 0;
    cJSON *root = cJSON_ParseWithLength(json, (size_t)len);
    if (!root) { ESP_LOGW(TAG, "json parse failed"); return false; }

    // WiFi: "w" = [[ssid,pass],...] → StackChan の SsidManager に追加(複数 SSID 可)
    int added = 0;
    const cJSON *w = cJSON_GetObjectItemCaseSensitive(root, "w");
    if (cJSON_IsArray(w)) {
        const cJSON *pair = nullptr;
        cJSON_ArrayForEach(pair, w) {
            const cJSON *ss = cJSON_GetArrayItem(pair, 0);
            const cJSON *pp = cJSON_GetArrayItem(pair, 1);
            if (cJSON_IsString(ss) && ss->valuestring && ss->valuestring[0]) {
                const char *pass = (cJSON_IsString(pp) && pp->valuestring) ? pp->valuestring : "";
                SsidManager::GetInstance().AddSsid(ss->valuestring, pass);
                added++;
                ESP_LOGI(TAG, "AddSsid: %s", ss->valuestring);
            }
        }
    }
    if (wifi_added) *wifi_added = added;

    // token: "t" → NVS
    bool tok_ok = false;
    const cJSON *t = cJSON_GetObjectItemCaseSensitive(root, "t");
    if (cJSON_IsString(t) && t->valuestring && t->valuestring[0]) {
        cat_store_set_token(t->valuestring);
        tok_ok = true;
        ESP_LOGI(TAG, "token saved (len=%d)", (int)strlen(t->valuestring));
    }

    // host: "h" = 1/2(index)or URL 文字列 → NVS
    const cJSON *hh = cJSON_GetObjectItemCaseSensitive(root, "h");
    if (cJSON_IsNumber(hh)) {
        cat_store_set_host(hh->valueint == 1 ? HOST_PROD : HOST_STAGING);
        ESP_LOGI(TAG, "host = %s", hh->valueint == 1 ? "prod" : "staging");
    } else if (cJSON_IsString(hh) && hh->valuestring && hh->valuestring[0]) {
        cat_store_set_host(hh->valuestring);
        ESP_LOGI(TAG, "host = %s", hh->valuestring);
    }

    cJSON_Delete(root);
    return tok_ok || added > 0;
}
