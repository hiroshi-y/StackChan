// cat_provision — QR の JSON を解釈してプロビジョニングする。
//   QR: {"v":1,"w":[[ssid,pass],...],"t":"token","h":<1|2|url>}
//   - w  → StackChan の SsidManager に AddSsid(WiFi 追加。複数可)
//   - t  → トークンを NVS(cat_store)へ
//   - h  → ホスト(1=本番/2=staging/URL)を NVS へ
#pragma once

// 成功で true。summary に人間可読の結果(WiFi名・トークン・ホスト)を埋める(画面表示用)。
bool cat_provision_apply_json(const char *json, int len, char *summary, int summary_len);
