// cat_bridge — Worker(worker-cat Durable Object)への WSS 中継。
// StackChan の既存 WiFi を流用し、wss://<host>/cat/device?token=... に接続。
// ブラウザ→DO から来た CAT バイト列を USB-VCP へ素通し、無線機の応答を WSS へ返す。
// cat-box の cat_bridge から WiFi 初期化部を除いて移植(WiFi は StackChan の HAL が管理)。
#pragma once

// ブリッジ開始: 無線機 RX をタップし、WiFi 確保 → WSS 接続を開始する。
void cat_bridge_start(void);

// ブリッジ停止: WSS を切断しタップを解除する(QUIT 時に呼ぶ)。
void cat_bridge_stop(void);

// 状態参照(画面表示用)。
bool cat_bridge_ws_connected(void);
const char *cat_bridge_last_err(void);
