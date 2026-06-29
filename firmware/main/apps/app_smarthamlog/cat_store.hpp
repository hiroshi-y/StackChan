// cat_store — ペアリング設定(トークン/ホスト)を NVS に保存・読み出し。
// QR ペアリングで書き込み、cat_bridge が読む。NVS に無ければ cat_config.h にフォールバック。
#pragma once

bool cat_store_get_token(char *out, int outlen);   // 保存トークン → out。あれば true
bool cat_store_get_host(char *out, int outlen);    // 保存ホスト → out。あれば true
void cat_store_set_token(const char *token);
void cat_store_set_host(const char *host);
