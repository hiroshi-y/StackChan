// cat_qr — カメラ(V4L2/esp_video)で QR を読み、デコードしたペイロードをコールバックへ渡す。
// cat-box の cat_qr から移植。cat_ui 依存を除去し、カメラは GREY / YUV422(Y抽出)両対応。
#pragma once
#include <stdint.h>

typedef void (*cat_qr_decoded_cb)(const char *payload, int len);

// スキャン開始(カメラ + quirc タスク起動)。on_decoded はデコード成功時に呼ばれる。
bool cat_qr_begin(cat_qr_decoded_cb on_decoded);

// スキャン停止(カメラ解放)。
void cat_qr_end(void);

// 状態文字列(画面表示用)。
const char *cat_qr_status(void);
uint32_t    cat_qr_frame_count(void);
uint32_t    cat_qr_detect_count(void);
