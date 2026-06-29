// cat_store — ペアリング設定(トークン/ホスト)の NVS 永続化。
// NVS 初期化は StackChan が起動時に済ませている前提(nvs_open がそのまま使える)。
#include "cat_store.hpp"
#include "nvs.h"
#include <cstring>

static const char *NS = "smarthamlog";

static bool get_str(const char *key, char *out, int outlen)
{
    if (outlen <= 0) return false;
    out[0] = '\0';
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t len = (size_t)outlen;
    esp_err_t e = nvs_get_str(h, key, out, &len);
    nvs_close(h);
    return (e == ESP_OK && out[0] != '\0');
}

static void set_str(const char *key, const char *val)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, key, val ? val : "");
    nvs_commit(h);
    nvs_close(h);
}

bool cat_store_get_token(char *out, int outlen) { return get_str("tok", out, outlen); }
bool cat_store_get_host(char *out, int outlen) { return get_str("host", out, outlen); }
void cat_store_set_token(const char *token) { set_str("tok", token); }
void cat_store_set_host(const char *host) { set_str("host", host); }
