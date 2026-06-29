// cat_qr — カメラ(V4L2/esp_video)→ quirc で QR デコード。cat-box から移植。
//  設計(cat-box の知見): スキャン中は内部スタックを小さく保ち(DVP DMA と競合させない)、
//  quirc identify+decode は同一タスク・PSRAM スタックで行う。局所適応二値化で読取安定化。
//  StackChan 対応: カメラフォーマットを GREY/YUV422(YUYV/UYVY)自動判定し、Y を抽出する。
#include "cat_qr.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "linux/videodev2.h"
#include "esp_video_device.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "quirc.h"

static const char *TAG = "cat_qr";

#define QR_W 320
#define QR_H 240
#define QR_BUF_COUNT 2
#define QR_DEV ESP_VIDEO_DVP_DEVICE_NAME
#define QR_DECODE_STACK 32768   // quirc_decode の datastream を収める(PSRAM スタック)

static int s_fd = -1;
static uint8_t *s_buf[QR_BUF_COUNT] = { nullptr, nullptr };
static size_t s_buf_len[QR_BUF_COUNT] = { 0, 0 };
static uint32_t s_pixfmt = 0;                  // 実フォーマット(V4L2_PIX_FMT_*)
static uint8_t *s_gray = nullptr;              // フレーム → グレースケール(PSRAM)
static struct quirc *s_q = nullptr;
static struct quirc_code *s_code = nullptr;
static struct quirc_data *s_data = nullptr;
static TaskHandle_t s_task = nullptr;
static StackType_t *s_decode_stack = nullptr;
static StaticTask_t s_decode_tcb;
static uint32_t *s_integral = nullptr;
static volatile bool s_run = false;
static volatile bool s_alive = false;
static volatile bool s_hit = false;
static cat_qr_decoded_cb s_decoded = nullptr;
static volatile uint32_t s_frames = 0;
static volatile uint32_t s_detect = 0;
static char s_status[40] = "";

uint32_t cat_qr_frame_count(void) { return s_frames; }
uint32_t cat_qr_detect_count(void) { return s_detect; }
const char *cat_qr_status(void) { return s_status; }
static void set_status(const char *s) { snprintf(s_status, sizeof(s_status), "%s", s); ESP_LOGI(TAG, "%s", s); }

// V4L2 ストリーム開始。GREY を要求し、ダメなら YUV422 を受ける(Y を抽出して使う)。
static bool stream_start(void)
{
    s_fd = open(QR_DEV, O_RDONLY);
    if (s_fd < 0) { set_status("camera open failed"); return false; }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = QR_W;
    fmt.fmt.pix.height = QR_H;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;   // まず GREY を要求
    ioctl(s_fd, VIDIOC_S_FMT, &fmt);
    // 実際に設定されたフォーマットを読み戻す(GREY 非対応なら YUV422 等になる)
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(s_fd, VIDIOC_G_FMT, &fmt) != 0) { set_status("G_FMT failed"); return false; }
    s_pixfmt = fmt.fmt.pix.pixelformat;
    ESP_LOGI(TAG, "pixfmt=0x%08x %dx%d", (unsigned)s_pixfmt, (int)fmt.fmt.pix.width, (int)fmt.fmt.pix.height);

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = QR_BUF_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(s_fd, VIDIOC_REQBUFS, &req) != 0) { set_status("REQBUFS failed"); return false; }

    for (int i = 0; i < QR_BUF_COUNT; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (ioctl(s_fd, VIDIOC_QUERYBUF, &buf) != 0) { set_status("QUERYBUF failed"); return false; }
        s_buf[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, s_fd, buf.m.offset);
        s_buf_len[i] = buf.length;
        if (s_buf[i] == MAP_FAILED) { s_buf[i] = nullptr; set_status("mmap failed"); return false; }
        if (ioctl(s_fd, VIDIOC_QBUF, &buf) != 0) { set_status("QBUF failed"); return false; }
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(s_fd, VIDIOC_STREAMON, &type) != 0) { set_status("STREAMON failed"); return false; }
    return true;
}

static void stream_stop(void)
{
    if (s_fd >= 0) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(s_fd, VIDIOC_STREAMOFF, &type);
    }
    for (int i = 0; i < QR_BUF_COUNT; i++) {
        if (s_buf[i]) { munmap(s_buf[i], s_buf_len[i]); s_buf[i] = nullptr; s_buf_len[i] = 0; }
    }
    if (s_fd >= 0) { close(s_fd); s_fd = -1; }
}

// フレーム(任意フォーマット)→ s_gray(8bit グレースケール QR_W*QR_H)。
static void to_gray(const uint8_t *frame, size_t len)
{
    const int N = QR_W * QR_H;
    if (s_pixfmt == V4L2_PIX_FMT_GREY) {
        memcpy(s_gray, frame, (size_t)N < len ? (size_t)N : len);
    } else if (s_pixfmt == V4L2_PIX_FMT_UYVY) {
        // U Y V Y ... → Y は奇数オフセット
        for (int i = 0; i < N && (size_t)(i * 2 + 1) < len; i++) s_gray[i] = frame[i * 2 + 1];
    } else {
        // YUYV(既定)/その他 → Y は偶数オフセット
        for (int i = 0; i < N && (size_t)(i * 2) < len; i++) s_gray[i] = frame[i * 2];
    }
}

// 局所適応二値化(Bradley 法 + 積分画像)。in(グレー) → out(0/255)。
static void adaptive_threshold(const uint8_t *in, uint8_t *out, int w, int h)
{
    if (!s_integral) { memcpy(out, in, (size_t)w * h); return; }
    const int IW = w + 1;
    uint32_t *I = s_integral;
    for (int x = 0; x <= w; x++) I[x] = 0;
    for (int y = 1; y <= h; y++) {
        uint32_t rowsum = 0;
        I[y * IW] = 0;
        for (int x = 1; x <= w; x++) {
            rowsum += in[(y - 1) * w + (x - 1)];
            I[y * IW + x] = I[(y - 1) * IW + x] + rowsum;
        }
    }
    const int R = 8;
    const int T = 15;
    for (int y = 0; y < h; y++) {
        int y0 = y - R < 0 ? 0 : y - R;
        int y1 = y + R >= h ? h - 1 : y + R;
        for (int x = 0; x < w; x++) {
            int x0 = x - R < 0 ? 0 : x - R;
            int x1 = x + R >= w ? w - 1 : x + R;
            int area = (y1 - y0 + 1) * (x1 - x0 + 1);
            uint32_t sum = I[(y1 + 1) * IW + (x1 + 1)] - I[y0 * IW + (x1 + 1)]
                         - I[(y1 + 1) * IW + x0] + I[y0 * IW + x0];
            int v = in[y * w + x];
            out[y * w + x] = ((long)v * 100 * area < (long)sum * (100 - T)) ? 0 : 255;
        }
    }
}

static bool decode_current(void);

static void scan_task(void *arg)
{
    (void)arg;
    s_alive = true;
    int errcnt = 0;
    while (s_run) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(s_fd, VIDIOC_DQBUF, &buf) != 0) {
            if (++errcnt > 30) { set_status("DQBUF failed"); break; }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        errcnt = 0;

        if (buf.flags & V4L2_BUF_FLAG_DONE) {
            s_frames++;
            to_gray(s_buf[buf.index], s_buf_len[buf.index]);
            if (!s_hit) {
                int w, h;
                uint8_t *img = quirc_begin(s_q, &w, &h);
                if (img) {
                    adaptive_threshold(s_gray, img, w, h);
                    quirc_end(s_q);
                    if (quirc_count(s_q) > 0) {
                        s_detect++;
                        if (decode_current()) s_hit = true;
                    }
                }
            }
        }
        ioctl(s_fd, VIDIOC_QBUF, &buf);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    s_alive = false;
    s_task = nullptr;
    vTaskDelete(NULL);
}

static bool decode_current(void)
{
    if (!s_q || !s_code || !s_data) return false;
    int n = quirc_count(s_q);
    for (int i = 0; i < n; i++) {
        quirc_extract(s_q, i, s_code);
        quirc_decode_error_t err = quirc_decode(s_code, s_data);
        if (err == QUIRC_ERROR_DATA_ECC) {
            quirc_flip(s_code);
            err = quirc_decode(s_code, s_data);
        }
        if (err == QUIRC_SUCCESS && s_data->payload_len > 0) {
            set_status("decoded");
            if (s_decoded) s_decoded((const char *)s_data->payload, (int)s_data->payload_len);
            return true;
        }
    }
    return false;
}

bool cat_qr_begin(cat_qr_decoded_cb on_decoded)
{
    if (s_run || s_alive) return false;
    s_decoded = on_decoded;
    s_frames = 0;
    s_detect = 0;
    s_hit = false;
    set_status("scanning");

    if (!stream_start()) { stream_stop(); return false; }

    s_q = quirc_new();
    if (!s_q || quirc_resize(s_q, QR_W, QR_H) < 0) {
        set_status("quirc alloc failed");
        if (s_q) { quirc_destroy(s_q); s_q = nullptr; }
        stream_stop();
        return false;
    }

    // 大きいバッファは内部RAMを圧迫しないよう PSRAM に確保(一度確保して再利用)。
    if (!s_gray)     s_gray = (uint8_t *)heap_caps_malloc(QR_W * QR_H, MALLOC_CAP_SPIRAM);
    s_code = (struct quirc_code *)heap_caps_malloc(sizeof(*s_code), MALLOC_CAP_SPIRAM);
    s_data = (struct quirc_data *)heap_caps_malloc(sizeof(*s_data), MALLOC_CAP_SPIRAM);
    if (!s_decode_stack) s_decode_stack = (StackType_t *)heap_caps_malloc(QR_DECODE_STACK, MALLOC_CAP_SPIRAM);
    if (!s_integral) s_integral = (uint32_t *)heap_caps_malloc((size_t)(QR_W + 1) * (QR_H + 1) * sizeof(uint32_t), MALLOC_CAP_SPIRAM);
    if (!s_gray || !s_code || !s_data || !s_decode_stack || !s_integral) {
        set_status("buffer alloc failed");
        free(s_code); s_code = nullptr;
        free(s_data); s_data = nullptr;
        quirc_destroy(s_q); s_q = nullptr;
        stream_stop();
        return false;
    }

    s_run = true;
    s_task = xTaskCreateStatic(scan_task, "cat_qr", QR_DECODE_STACK, nullptr, 5, s_decode_stack, &s_decode_tcb);
    if (!s_task) {
        s_run = false;
        free(s_code); s_code = nullptr;
        free(s_data); s_data = nullptr;
        quirc_destroy(s_q); s_q = nullptr;
        stream_stop();
        set_status("task create failed");
        return false;
    }
    return true;
}

void cat_qr_end(void)
{
    s_run = false;
    for (int i = 0; i < 200 && s_alive; i++) vTaskDelay(pdMS_TO_TICKS(10));
    vTaskDelay(pdMS_TO_TICKS(30));
    stream_stop();
    if (s_q) { quirc_destroy(s_q); s_q = nullptr; }
    free(s_code); s_code = nullptr;
    free(s_data); s_data = nullptr;
    s_hit = false;
    s_decoded = nullptr;
}
