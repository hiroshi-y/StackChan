// cat_bridge — Worker(worker-cat DO)への WSS 中継(StackChan 版)。
//   StackChan の既存 WiFi を流用し wss://<host>/cat/device?token=... に接続。
//   ブラウザ→DO から来た CAT バイト列を USB-VCP へ素通し、無線機の応答を WSS へ返す。
//   cat-box の cat_bridge から WiFi 初期化/イベント部を除去(WiFi は HAL が管理)。
#include "cat_bridge.hpp"
#include "cat_core.hpp"
#include "cat_store.hpp"   // NVS のトークン/ホスト(QR ペアリング)
#include "hal/hal.h"   // GetHAL(), WifiStatus

#if __has_include("cat_config.h")
#include "cat_config.h"
#else
#include "cat_config.example.h"
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_websocket_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include "mbedtls/base64.h"
#include "esp_heap_caps.h"

#include <cstring>
#include <cstdio>
#include <string_view>

static const char *TAG = "cat_bridge";

static bool s_started = false;
static volatile bool s_stop = false;
static bool s_ws_up = false;
static esp_websocket_client_handle_t s_client = nullptr;
static char s_last_err[64] = "";

// ---- 無線機 → WSS (生 RX を base64 で {"t":"rx","d":...} として送る) ----
static void on_radio_rx(const uint8_t *data, size_t len)
{
    if (!s_client || !s_ws_up || len == 0) return;
    unsigned char b64[512];
    size_t olen = 0;
    if (mbedtls_base64_encode(b64, sizeof(b64), &olen, data, len) != 0) return;
    char msg[640];
    int n = snprintf(msg, sizeof(msg), "{\"t\":\"rx\",\"d\":\"%.*s\"}", (int)olen, (const char *)b64);
    if (n > 0) esp_websocket_client_send_text(s_client, msg, n, pdMS_TO_TICKS(200));
}

// ---- WSS → 無線機 (ブラウザからの envelope を処理) ----
static void handle_text(const char *data, int len)
{
    cJSON *root = cJSON_ParseWithLength(data, (size_t)len);
    if (!root) return;
    const cJSON *t = cJSON_GetObjectItemCaseSensitive(root, "t");
    if (cJSON_IsString(t) && t->valuestring) {
        if (strcmp(t->valuestring, "open") == 0) {
            const cJSON *baud = cJSON_GetObjectItemCaseSensitive(root, "baud");
            if (cJSON_IsNumber(baud) && baud->valueint > 0) cat_core_set_baud((uint32_t)baud->valueint);
            cat_core_start();   // USB ホスト起動(冪等)
            ESP_LOGI(TAG, "WS open");
        } else if (strcmp(t->valuestring, "tx") == 0) {
            const cJSON *d = cJSON_GetObjectItemCaseSensitive(root, "d");
            if (cJSON_IsString(d) && d->valuestring) {
                unsigned char raw[256];
                size_t olen = 0;
                if (mbedtls_base64_decode(raw, sizeof(raw), &olen,
                        (const unsigned char *)d->valuestring, strlen(d->valuestring)) == 0 && olen > 0) {
                    cat_core_write_raw(raw, olen);
                }
            }
        } else if (strcmp(t->valuestring, "ping") == 0) {
            const char *pong = "{\"t\":\"pong\"}";
            esp_websocket_client_send_text(s_client, pong, (int)strlen(pong), pdMS_TO_TICKS(200));
        }
    }
    cJSON_Delete(root);
}

// WS エラー詳細(ハンドシェイク HTTP ステータス / TLS / ソケット)。
//   http=401 → ペア鍵不正, 404 → パス/ホスト違い, 426 → upgrade 無し
//   tls!=0   → 証明書/TLS 失敗, sock!=0 → 接続失敗(DNS/到達不可)
static void log_ws_err(esp_websocket_event_data_t *d)
{
    if (!d) return;
    ESP_LOGW(TAG, "ws err http=%d type=%d tls=0x%x sock=%d",
             d->error_handle.esp_ws_handshake_status_code,
             (int)d->error_handle.error_type,
             (unsigned)d->error_handle.esp_tls_last_esp_err,
             d->error_handle.esp_transport_sock_errno);
    snprintf(s_last_err, sizeof(s_last_err), "http%d tls0x%x sk%d",
             d->error_handle.esp_ws_handshake_status_code,
             (unsigned)d->error_handle.esp_tls_last_esp_err,
             d->error_handle.esp_transport_sock_errno);
}

// ---- WebSocket イベント ----
static void ws_event(void *arg, esp_event_base_t base, int32_t id, void *event_data)
{
    (void)arg; (void)base;
    esp_websocket_event_data_t *d = (esp_websocket_event_data_t *)event_data;
    switch (id) {
        case WEBSOCKET_EVENT_CONNECTED:
            s_ws_up = true; s_last_err[0] = '\0'; ESP_LOGI(TAG, "WS connected");
            break;
        case WEBSOCKET_EVENT_DISCONNECTED:
            s_ws_up = false; log_ws_err(d);
            break;
        case WEBSOCKET_EVENT_DATA:
            if (d && d->op_code == 0x01 && d->data_len > 0 && d->data_ptr) {
                handle_text(d->data_ptr, d->data_len);
            }
            break;
        case WEBSOCKET_EVENT_ERROR:
            log_ws_err(d); break;
        default: break;
    }
}

static void ws_start(void)
{
    if (s_client) return;
    // ホスト/トークンは NVS(QR ペアリングで設定)優先、無ければ cat_config.h。
    static char host[128], token[160];
    if (!cat_store_get_host(host, sizeof(host)))   snprintf(host, sizeof(host), "%s", CAT_WORKER_HOST);
    if (!cat_store_get_token(token, sizeof(token))) snprintf(token, sizeof(token), "%s", CAT_PAIR_TOKEN);
    static char uri[320];
    snprintf(uri, sizeof(uri), "wss://%s/cat/device?token=%s", host, token);

    esp_websocket_client_config_t cfg = {};
    cfg.uri = uri;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;   // 公開CA(Cloudflare)を検証
    cfg.reconnect_timeout_ms = 5000;
    cfg.network_timeout_ms = 10000;
    cfg.task_stack = 12288;  // TLS ハンドシェイク(証明書バンドル)用に増やす(401 リトライ中の安定化)

    s_client = esp_websocket_client_init(&cfg);
    if (!s_client) { ESP_LOGE(TAG, "ws init failed"); snprintf(s_last_err, sizeof(s_last_err), "ws init fail"); return; }
    esp_websocket_register_events(s_client, WEBSOCKET_EVENT_ANY, ws_event, nullptr);
    ESP_LOGI(TAG, "-> wss://%s/cat/device", host);
    esp_websocket_client_start(s_client);
}

// WiFi を確保してから WSS を開始するタスク。
static void bridge_task(void *arg)
{
    // WiFi が未接続なら StackChan の HAL で接続開始(既に上がっていれば呼ばない)。
    if (GetHAL().getWifiStatus() == WifiStatus::None) {
        GetHAL().startNetwork([](std::string_view m) {
            ESP_LOGI(TAG, "net: %.*s", (int)m.size(), m.data());
        });
    }
    // 接続待ち(最大 ~25s)。
    int waited = 0;
    while (!s_stop && GetHAL().getWifiStatus() == WifiStatus::None && waited < 25000) {
        vTaskDelay(pdMS_TO_TICKS(500));
        waited += 500;
    }
    if (!s_stop && GetHAL().getWifiStatus() != WifiStatus::None) {
        ws_start();
    } else if (!s_stop) {
        snprintf(s_last_err, sizeof(s_last_err), "wifi timeout");
        ESP_LOGW(TAG, "wifi timeout");
    }
    vTaskDelete(nullptr);
}

void cat_bridge_start(void)
{
    if (s_started) return;
    s_started = true;
    s_stop = false;
    s_ws_up = false;
    cat_core_set_raw_rx(on_radio_rx);   // 無線機 RX → WSS
    xTaskCreate(bridge_task, "cat_bridge", 5120, nullptr, 5, nullptr);
}

void cat_bridge_stop(void)
{
    if (!s_started) return;
    s_stop = true;
    cat_core_set_raw_rx(nullptr);
    if (s_client) {
        esp_websocket_client_stop(s_client);
        esp_websocket_client_destroy(s_client);
        s_client = nullptr;
    }
    s_ws_up = false;
    s_started = false;
}

bool cat_bridge_ws_connected(void) { return s_ws_up; }
const char *cat_bridge_last_err(void) { return s_last_err; }
