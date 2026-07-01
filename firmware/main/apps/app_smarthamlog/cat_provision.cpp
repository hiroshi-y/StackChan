// cat_provision — QR ペイロードを解釈してプロビジョニング(cat-box の cat_prov_save_json 相当)。
//   2形式:
//    - コンパクト(現行): "<hostidx>\n<ssid>\n<pass>\n<token_b64>"
//    - JSON(後方互換):  {"v":1,"w":[[ssid,pass]],"t":"<b64>","h":<1|2>}
//   WiFi は 1 件のみ(QR を小さく保ち低解像度カメラで読めるようにするため。複数は載せない)。
//   トークンは base64 で来る。復号して 32 バイトなら hex(64文字)に戻す(worker は 64-hex 期待)。
#include "cat_provision.hpp"
#include "cat_store.hpp"
#include "cat_hal.h"   // cat_hal_wifi_add_ssid(WiFi 認証情報の保存)。機種ごとに cat_hal.cpp で実装。
#include "cJSON.h"
#include "mbedtls/base64.h"
#include "esp_log.h"
#include <cstring>
#include <cstdlib>

static const char *TAG = "cat_prov";

static const char *HOST_PROD    = "smart-hamlog-cat.hiroshi-a4b.workers.dev";
static const char *HOST_STAGING = "staging-smart-hamlog-cat.hiroshi-a4b.workers.dev";

bool cat_provision_apply_json(const char *json, int len, char *summary, int summary_len)
{
    if (summary && summary_len > 0) summary[0] = '\0';
    char ssid[33] = {0}, pass[64] = {0}, tokbuf[160] = {0};
    int  host_idx = 0;
    bool parsed = false;

    if (len > 0 && json[0] == '{') {
        // JSON(後方互換)
        cJSON *root = cJSON_ParseWithLength(json, (size_t)len);
        if (root) {
            const cJSON *w  = cJSON_GetObjectItemCaseSensitive(root, "w");
            const cJSON *t  = cJSON_GetObjectItemCaseSensitive(root, "t");
            const cJSON *hh = cJSON_GetObjectItemCaseSensitive(root, "h");
            if (cJSON_IsArray(w)) {
                const cJSON *pair = nullptr;
                cJSON_ArrayForEach(pair, w) {
                    const cJSON *ss = cJSON_GetArrayItem(pair, 0);
                    const cJSON *pp = cJSON_GetArrayItem(pair, 1);
                    if (cJSON_IsString(ss) && ss->valuestring && ss->valuestring[0]) {
                        snprintf(ssid, sizeof(ssid), "%s", ss->valuestring);
                        snprintf(pass, sizeof(pass), "%s",
                                 (cJSON_IsString(pp) && pp->valuestring) ? pp->valuestring : "");
                        break;
                    }
                }
            }
            if (cJSON_IsString(t) && t->valuestring) snprintf(tokbuf, sizeof(tokbuf), "%s", t->valuestring);
            if (cJSON_IsNumber(hh)) host_idx = hh->valueint;
            parsed = (ssid[0] != 0);
            cJSON_Delete(root);
        }
    } else {
        // コンパクト: hostIdx \n ssid \n pass \n token_b64
        char tmp[256];
        int n = len < (int)sizeof(tmp) - 1 ? len : (int)sizeof(tmp) - 1;
        memcpy(tmp, json, n);
        tmp[n] = '\0';
        char *f[4] = {0};
        int fi = 0;
        f[fi++] = tmp;
        for (char *p = tmp; *p && fi < 4; p++) {
            if (*p == '\n') { *p = '\0'; f[fi++] = p + 1; }
        }
        if (fi >= 4 && f[1][0]) {
            host_idx = atoi(f[0]);
            snprintf(ssid, sizeof(ssid), "%s", f[1]);
            snprintf(pass, sizeof(pass), "%s", f[2]);
            snprintf(tokbuf, sizeof(tokbuf), "%s", f[3]);
            parsed = true;
        }
    }

    if (!parsed || !ssid[0]) { ESP_LOGW(TAG, "bad format"); return false; }

    // WiFi 認証情報を保存(実装は機種依存: StackChan=SsidManager / cat-box=独自)。
    cat_hal_wifi_add_ssid(ssid, pass);
    ESP_LOGI(TAG, "AddSsid: %s", ssid);

    // トークン: base64 → 32 バイトなら hex(64文字)に戻す(worker は 64-hex 期待)。
    const char *tok_str = tokbuf;
    char tok_hex[65];
    {
        unsigned char raw[48];
        size_t rawlen = 0;
        if (tokbuf[0] &&
            mbedtls_base64_decode(raw, sizeof(raw), &rawlen,
                                  (const unsigned char *)tokbuf, strlen(tokbuf)) == 0 && rawlen == 32) {
            static const char hx[] = "0123456789abcdef";
            for (int i = 0; i < 32; i++) {
                tok_hex[i * 2]     = hx[(raw[i] >> 4) & 0xf];
                tok_hex[i * 2 + 1] = hx[raw[i] & 0xf];
            }
            tok_hex[64] = '\0';
            tok_str = tok_hex;
        }
    }
    if (tok_str[0]) { cat_store_set_token(tok_str); ESP_LOGI(TAG, "token saved (len=%d)", (int)strlen(tok_str)); }

    // ホスト: 1=本番 / 2=staging
    if (host_idx == 1)      cat_store_set_host(HOST_PROD);
    else if (host_idx == 2) cat_store_set_host(HOST_STAGING);
    ESP_LOGI(TAG, "host_idx=%d", host_idx);

    if (summary && summary_len > 0) {
        snprintf(summary, summary_len, "WiFi:%s%s host:%s", ssid,
                 tokbuf[0] ? " +token" : "",
                 host_idx == 1 ? "prod" : host_idx == 2 ? "staging" : "(keep)");
    }
    return true;
}
