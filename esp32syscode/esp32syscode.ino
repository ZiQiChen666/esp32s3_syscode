/*
 * ============================================================================
 *  DNESP32S3 智能网络摄像头 (OV2640 + Web + SD/FATFS)
 * ----------------------------------------------------------------------------
 *  开发板 : 正点原子 DNESP32S3 (ATK_DNESP32S3 V1.2, ESP32-S3-WROOM-1)
 *  功能   :
 *    1) WiFi 连接 + 网页实时预览 (MJPEG)
 *    2) 网页拍照并保存到 SD 卡 (FATFS)
 *    3) 网页录制 MJPEG-AVI 视频并保存到 SD 卡
 *    4) 网页浏览 / 下载 / 删除 SD 卡文件
 *    5) 网页调节基本摄像头参数 (分辨率/画质/亮度/对比度/白平衡/曝光...)
 *  WiFi   : SSID = abc , 密码 = 123456789b
 *
 *  说明:
 *    - 不使用 LCD、蜂鸣器、音频。仅通过 XL9555 的 P0.4/P0.5 给摄像头上电。
 *    - SD 卡工作在 SPI 模式: CS=IO2, SCK=IO12, MISO=IO13, MOSI=IO11，
 *      SD 库底层即 FatFs，挂载点 /sd，即 FATFS 文件系统。
 *    - 录像格式为 MJPEG 封装的 AVI，可直接用 VLC / PotPlayer 播放。
 *
 *  Arduino IDE 设置:
 *    开发板: ESP32S3 Dev Module | PSRAM: OPI PSRAM | Flash: 16MB
 *    分区表: Huge APP (3MB No OTA/1MB SPIFFS)
 * ============================================================================
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "esp_http_server.h"
#include "img_converters.h"
#include <vector>

// ============================ WiFi ============================
static const char *WIFI_SSID = "abc";
static const char *WIFI_PASS = "123456789b";

// ==================== 摄像头引脚 (DNESP32S3) ====================
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    -1
#define CAM_PIN_SIOD    39
#define CAM_PIN_SIOC    38
#define CAM_PIN_D7      18
#define CAM_PIN_D6      17
#define CAM_PIN_D5      16
#define CAM_PIN_D4      15
#define CAM_PIN_D3      7
#define CAM_PIN_D2      6
#define CAM_PIN_D1      5
#define CAM_PIN_D0      4
#define CAM_PIN_VSYNC   47
#define CAM_PIN_HREF    48
#define CAM_PIN_PCLK    45

// ==================== SD 卡引脚 (SPI) ====================
#define SD_CS_PIN       2
#define SD_SCK_PIN      12
#define SD_MISO_PIN     13
#define SD_MOSI_PIN     11

#define PHOTO_DIR       "/photo"
#define VIDEO_DIR       "/video"
#define REC_FPS         10

// ==================== XL9555 ====================
#define IIC_SDA_PIN     41
#define IIC_SCL_PIN     42
#define XL9555_ADDR     0x20
#define XL9555_REG_OUT0 0x02
#define XL9555_REG_CFG0 0x06
#define XL9555_REG_OUT1 0x03
#define XL9555_REG_CFG1 0x07
#define OV_PWDN_BIT     4
#define OV_RESET_BIT    5
// XL9555 P1 上的 LCD 控制位
#define LCD_BL_BIT      0   // P1.0 背光
#define LCD_RST_BIT     2   // P1.2 SPI LCD 复位
#define LCD_PWR_BIT     3   // P1.3 SPI LCD 电源

// ==================== SPI LCD (可选镜像显示) ====================
// 注意: SPI LCD 与 SD 卡共用 SPI2 (IO11/12/13)，代码内部用互斥锁串行化。
#define LCD_ENABLE_DEFAULT  0     // 上电默认是否打开 LCD
#define LCD_IS_ST7789       1     // 1=ST7789  0=ILI9341
// ---- 分辨率: 按你的 ST7789 面板选一个 ----
//   2.4" 240x320 横屏: LCD_W 320, LCD_H 240, LCD_OFFSET_X 0, LCD_OFFSET_Y 0
//   1.3" 240x240 横屏: LCD_W 240, LCD_H 240, LCD_OFFSET_X 0, LCD_OFFSET_Y 80
//   1.14"135x240 横屏: LCD_W 240, LCD_H 135, LCD_OFFSET_X 40, LCD_OFFSET_Y 52
#define LCD_W               320   // 逻辑分辨率(横屏)
#define LCD_H               240
#define LCD_FRAMESIZE       FRAMESIZE_QVGA  // 必须与 LCD_W/LCD_H 匹配(240x240 用 FRAMESIZE_240X240)
#define LCD_CS_PIN          21    // SLCD_CS
#define LCD_DC_PIN          40    // LCD_DC
#define LCD_SCK_PIN         SD_SCK_PIN
#define LCD_MOSI_PIN        SD_MOSI_PIN
#define LCD_MADCTL          0x60  // 横屏; 方向不对改 0x00/0x60/0x70/0xA0
#define LCD_SWAP_BYTES      1     // RGB565 字节序不对时改 0
#define LCD_INVERT          1     // 颜色反相; 不对时改 0
#define LCD_OFFSET_X        0     // 面板偏移
#define LCD_OFFSET_Y        0
#define LCD_SPI_HZ          20000000

// ============================ 全局 ============================
static httpd_handle_t s_server = NULL;
static httpd_handle_t s_streamServer = NULL;
static bool s_sdReady = false;
static volatile bool s_camBusy = false;   // 拍照时短暂暂停视频流，避免争用帧缓冲
static volatile bool s_lcdOn = false;     // LCD 开关状态
static volatile bool s_lcdRunning = false;
static SemaphoreHandle_t s_spiMutex = NULL;   // SPI2 总线互斥(SD 与 LCD 共用)

#define SPI_LOCK()   do { if (s_spiMutex) xSemaphoreTake(s_spiMutex, portMAX_DELAY); } while (0)
#define SPI_UNLOCK() do { if (s_spiMutex) xSemaphoreGive(s_spiMutex); } while (0)

// ---- AVI 录制状态 ----
static File s_avi;
static volatile bool s_recActive = false;
static volatile bool s_recRunning = false;
static bool s_headerWritten = false;
static uint32_t s_recFrames = 0;
static uint32_t s_recStartMs = 0;
static uint32_t s_recMaxFrame = 0;
static uint32_t s_moviStartPos = 0;
static uint32_t s_posRiffSize = 0;
static uint32_t s_posAvihFrames = 0;
static uint32_t s_posAvihMaxBytes = 0;
static uint32_t s_posAvihSuggested = 0;
static uint32_t s_posStrhLength = 0;
static uint32_t s_posStrhSuggested = 0;
static uint32_t s_posMoviListSize = 0;
static uint32_t s_posStrfImage = 0;
static String s_recFile;
static std::vector<uint32_t> s_frameOffsets;
static std::vector<uint32_t> s_frameSizes;

// ============================ 工具函数 ============================
static void send_json(httpd_req_t *req, const String &body) {
  httpd_resp_set_type(req, "application/json; charset=utf-8");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, body.c_str(), body.length());
}

static void send_ok(httpd_req_t *req) { send_json(req, "{\"ok\":true}"); }
static void send_err(httpd_req_t *req, const char *msg) {
  send_json(req, String("{\"ok\":false,\"error\":\"") + msg + "\"}");
}

// URL 解码 (%XX 与 +)
static void url_decode(char *dst, const char *src, size_t dstSize) {
  size_t i = 0;
  while (*src && i + 1 < dstSize) {
    if (*src == '%' && isxdigit((unsigned char)src[1]) && isxdigit((unsigned char)src[2])) {
      char h[3] = { src[1], src[2], 0 };
      dst[i++] = (char)strtol(h, NULL, 16);
      src += 3;
    } else if (*src == '+') {
      dst[i++] = ' '; src++;
    } else {
      dst[i++] = *src++;
    }
  }
  dst[i] = 0;
}

static bool ends_with_ci(const char *s, const char *suffix) {
  size_t ls = strlen(s), lf = strlen(suffix);
  if (lf > ls) return false;
  const char *p = s + (ls - lf);
  for (size_t i = 0; i < lf; i++) {
    char a = p[i], b = suffix[i];
    if (a >= 'A' && a <= 'Z') a += 32;
    if (b >= 'A' && b <= 'Z') b += 32;
    if (a != b) return false;
  }
  return true;
}

static const char *mime_for(const char *path) {
  if (ends_with_ci(path, ".jpg") || ends_with_ci(path, ".jpeg")) return "image/jpeg";
  if (ends_with_ci(path, ".avi")) return "video/x-msvideo";
  if (ends_with_ci(path, ".png")) return "image/png";
  return "application/octet-stream";
}

static void writeLE32(File &f, uint32_t v) {
  uint8_t b[4] = { (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF),
                   (uint8_t)((v >> 16) & 0xFF), (uint8_t)((v >> 24) & 0xFF) };
  f.write(b, 4);
}
static void writeLE16(File &f, uint16_t v) {
  uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF) };
  f.write(b, 2);
}
static void patchLE32(File &f, uint32_t pos, uint32_t v) {
  f.seek(pos);
  writeLE32(f, v);
}

// ==================== XL9555 摄像头供电 ====================
static bool xl9555_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(XL9555_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool xl9555_read(uint8_t reg, uint8_t *val) {
  Wire.beginTransmission(XL9555_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)XL9555_ADDR, 1) != 1) return false;
  *val = Wire.read();
  return true;
}
static void camera_power_on() {
  Wire.begin(IIC_SDA_PIN, IIC_SCL_PIN, 100000);
  uint8_t cfg = 0xFF;
  if (!xl9555_read(XL9555_REG_CFG0, &cfg)) {
    Serial.println("[XL9555] 读取失败 (IO41/IO42, 0x20)");
    return;
  }
  cfg &= ~((1 << OV_PWDN_BIT) | (1 << OV_RESET_BIT));
  xl9555_write(XL9555_REG_CFG0, cfg);
  uint8_t out = 0x00;
  xl9555_read(XL9555_REG_OUT0, &out);
  out &= ~(1 << OV_PWDN_BIT);
  out |=  (1 << OV_RESET_BIT);
  xl9555_write(XL9555_REG_OUT0, out);
  delay(20);
  Serial.println("[XL9555] 摄像头已上电");
}

// ==================== AVI 头 ====================
static void avi_write_header(File &f, int width, int height, int fps) {
  f.write((const uint8_t *)"RIFF", 4);
  s_posRiffSize = f.position(); writeLE32(f, 0);
  f.write((const uint8_t *)"AVI ", 4);

  f.write((const uint8_t *)"LIST", 4);
  uint32_t hdrlSizePos = f.position(); writeLE32(f, 0);
  f.write((const uint8_t *)"hdrl", 4);

  // avih
  f.write((const uint8_t *)"avih", 4);
  writeLE32(f, 56);
  writeLE32(f, 1000000 / fps);
  s_posAvihMaxBytes = f.position(); writeLE32(f, 0);
  writeLE32(f, 0);
  writeLE32(f, 0x10); // AVIF_HASINDEX
  s_posAvihFrames = f.position(); writeLE32(f, 0);
  writeLE32(f, 0);
  writeLE32(f, 1);
  s_posAvihSuggested = f.position(); writeLE32(f, 0);
  writeLE32(f, width);
  writeLE32(f, height);
  for (int i = 0; i < 4; i++) writeLE32(f, 0);

  // strl
  f.write((const uint8_t *)"LIST", 4);
  uint32_t strlSizePos = f.position(); writeLE32(f, 0);
  f.write((const uint8_t *)"strl", 4);

  // strh
  f.write((const uint8_t *)"strh", 4);
  writeLE32(f, 56);
  f.write((const uint8_t *)"vids", 4);
  f.write((const uint8_t *)"MJPG", 4);
  writeLE32(f, 0);
  writeLE16(f, 0); writeLE16(f, 0);
  writeLE32(f, 0);
  writeLE32(f, 1);
  writeLE32(f, fps);
  writeLE32(f, 0);
  s_posStrhLength = f.position(); writeLE32(f, 0);
  s_posStrhSuggested = f.position(); writeLE32(f, 0);
  writeLE32(f, 0);
  writeLE32(f, 0);
  writeLE16(f, 0); writeLE16(f, 0); writeLE16(f, width); writeLE16(f, height);

  // strf
  f.write((const uint8_t *)"strf", 4);
  writeLE32(f, 40);
  writeLE32(f, 40);
  writeLE32(f, width);
  writeLE32(f, height);
  writeLE16(f, 1);
  writeLE16(f, 24);
  f.write((const uint8_t *)"MJPG", 4);
  s_posStrfImage = f.position(); writeLE32(f, 0);
  writeLE32(f, 0); writeLE32(f, 0); writeLE32(f, 0); writeLE32(f, 0);

  uint32_t afterHdrl = f.position();
  patchLE32(f, hdrlSizePos, afterHdrl - hdrlSizePos - 4);
  uint32_t afterStrl = afterHdrl;
  patchLE32(f, strlSizePos, afterStrl - strlSizePos - 4);
  f.seek(afterHdrl);

  // movi
  f.write((const uint8_t *)"LIST", 4);
  s_posMoviListSize = f.position(); writeLE32(f, 0);
  s_moviStartPos = f.position();
  f.write((const uint8_t *)"movi", 4);
}

static void avi_finalize(File &f) {
  uint32_t fileEnd = f.position();
  uint32_t moviBytes = fileEnd - (s_moviStartPos + 4);

  patchLE32(f, s_posMoviListSize, fileEnd - s_moviStartPos);
  patchLE32(f, s_posAvihFrames, s_recFrames);
  patchLE32(f, s_posAvihMaxBytes, s_recFrames ? (moviBytes * REC_FPS / s_recFrames) : 0);
  patchLE32(f, s_posAvihSuggested, s_recMaxFrame + 8);
  patchLE32(f, s_posStrhLength, s_recFrames);
  patchLE32(f, s_posStrhSuggested, s_recMaxFrame + 8);
  patchLE32(f, s_posStrfImage, s_recMaxFrame);
  f.seek(fileEnd);

  // idx1
  f.write((const uint8_t *)"idx1", 4);
  writeLE32(f, s_recFrames * 16);
  for (uint32_t i = 0; i < s_recFrames; i++) {
    f.write((const uint8_t *)"00dc", 4);
    writeLE32(f, 0x10);
    writeLE32(f, s_frameOffsets[i]);
    writeLE32(f, s_frameSizes[i]);
  }

  uint32_t finalSize = f.position();
  patchLE32(f, s_posRiffSize, finalSize - 8);
  f.seek(finalSize);
}

// ==================== 录像任务 ====================
static void record_task(void *arg) {
  uint32_t interval = 1000 / REC_FPS;
  uint32_t last = millis();
  while (s_recActive) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      if (!s_headerWritten) {
        SPI_LOCK();
        avi_write_header(s_avi, fb->width, fb->height, REC_FPS);
        SPI_UNLOCK();
        s_headerWritten = true;
      }
      uint32_t pos = s_avi.position();
      SPI_LOCK();
      s_avi.write((const uint8_t *)"00dc", 4);
      writeLE32(s_avi, fb->len);
      s_avi.write(fb->buf, fb->len);
      if (fb->len & 1) { uint8_t z = 0; s_avi.write(&z, 1); }
      SPI_UNLOCK();
      s_frameOffsets.push_back(pos - s_moviStartPos);
      s_frameSizes.push_back(fb->len);
      if (fb->len > s_recMaxFrame) s_recMaxFrame = fb->len;
      s_recFrames++;
      esp_camera_fb_return(fb);
    }
    uint32_t now = millis();
    uint32_t elapsed = now - last;
    if (elapsed < interval) vTaskDelay(interval - elapsed);
    last = millis();
  }
  // 在录制任务内部完成 AVI 收尾, 避免与停止请求竞争文件句柄
  SPI_LOCK();
  if (!s_headerWritten) avi_write_header(s_avi, 320, 240, REC_FPS);
  avi_finalize(s_avi);
  s_avi.close();
  SPI_UNLOCK();
  s_recRunning = false;
  vTaskDelete(NULL);
}

// ==================== 文件工具 ====================
static int next_index(const char *dir, const char *prefix) {
  File d = SD.open(dir);
  if (!d) return 1;
  int maxN = 0;
  size_t plen = strlen(prefix);
  File f = d.openNextFile();
  while (f) {
    const char *full = f.name();
    if (full) {
      const char *base = strrchr(full, '/');
      base = base ? base + 1 : full;
      if (strncmp(base, prefix, plen) == 0) {
        int n = atoi(base + plen);
        if (n > maxN) maxN = n;
      }
    }
    f = d.openNextFile();
  }
  d.close();
  return maxN + 1;
}

static String humanSize(uint32_t s) {
  char b[32];
  if (s < 1024) snprintf(b, sizeof(b), "%u B", s);
  else if (s < 1024 * 1024) snprintf(b, sizeof(b), "%.1f KB", s / 1024.0);
  else snprintf(b, sizeof(b), "%.1f MB", s / 1048576.0);
  return String(b);
}

static void append_dir_json(String &out, const char *dir, const char *kind, bool &first) {
  File d = SD.open(dir);
  if (!d) return;
  File f = d.openNextFile();
  while (f) {
    if (!f.isDirectory()) {
      const char *full = f.name();
      const char *base = strrchr(full, '/');
      base = base ? base + 1 : full;
      String path = String(dir) + "/" + base;
      if (!first) out += ",";
      first = false;
      out += "{\"name\":\"" + String(base) + "\",\"path\":\"" + path +
             "\",\"size\":" + String((uint32_t)f.size()) +
             ",\"sizeText\":\"" + humanSize(f.size()) +
             "\",\"kind\":\"" + kind + "\"}";
    }
    f = d.openNextFile();
  }
  d.close();
}

// ==================== 摄像头参数 ====================
static int set_camera_param(const char *variable, int val) {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return -1;
  if (!strcmp(variable, "framesize")) {
    if (s_recActive) return -1;
    if (s->pixformat == PIXFORMAT_JPEG) return s->set_framesize(s, (framesize_t)val);
    return -1;
  }
  else if (!strcmp(variable, "quality"))        return s->set_quality(s, val);
  else if (!strcmp(variable, "contrast"))       return s->set_contrast(s, val);
  else if (!strcmp(variable, "brightness"))     return s->set_brightness(s, val);
  else if (!strcmp(variable, "saturation"))     return s->set_saturation(s, val);
  else if (!strcmp(variable, "sharpness"))      return s->set_sharpness(s, val);
  else if (!strcmp(variable, "gainceiling"))    return s->set_gainceiling(s, (gainceiling_t)val);
  else if (!strcmp(variable, "colorbar"))       return s->set_colorbar(s, val);
  else if (!strcmp(variable, "awb"))            return s->set_whitebal(s, val);
  else if (!strcmp(variable, "agc"))            return s->set_gain_ctrl(s, val);
  else if (!strcmp(variable, "aec"))            return s->set_exposure_ctrl(s, val);
  else if (!strcmp(variable, "hmirror"))        return s->set_hmirror(s, val);
  else if (!strcmp(variable, "vflip"))          return s->set_vflip(s, val);
  else if (!strcmp(variable, "awb_gain"))       return s->set_awb_gain(s, val);
  else if (!strcmp(variable, "agc_gain"))       return s->set_agc_gain(s, val);
  else if (!strcmp(variable, "aec_value"))      return s->set_aec_value(s, val);
  else if (!strcmp(variable, "aec2"))           return s->set_aec2(s, val);
  else if (!strcmp(variable, "dcw"))            return s->set_dcw(s, val);
  else if (!strcmp(variable, "bpc"))            return s->set_bpc(s, val);
  else if (!strcmp(variable, "wpc"))            return s->set_wpc(s, val);
  else if (!strcmp(variable, "raw_gma"))        return s->set_raw_gma(s, val);
  else if (!strcmp(variable, "lenc"))           return s->set_lenc(s, val);
  else if (!strcmp(variable, "special_effect")) return s->set_special_effect(s, val);
  else if (!strcmp(variable, "wb_mode"))        return s->set_wb_mode(s, val);
  else if (!strcmp(variable, "ae_level"))       return s->set_ae_level(s, val);
  return -1;
}

static String camera_status_json() {
  sensor_t *s = esp_camera_sensor_get();
  String j = "{";
  j += "\"sd\":" + String(s_sdReady ? "true" : "false");
  j += ",\"recording\":" + String(s_recActive ? "true" : "false");
  j += ",\"frames\":" + String(s_recFrames);
  j += ",\"seconds\":" + String(s_recActive ? (millis() - s_recStartMs) / 1000 : 0);
  j += ",\"lcd\":" + String(s_lcdOn ? "true" : "false");
  if (s) {
    j += ",\"framesize\":" + String((int)s->status.framesize);
    j += ",\"quality\":" + String(s->status.quality);
    j += ",\"brightness\":" + String(s->status.brightness);
    j += ",\"contrast\":" + String(s->status.contrast);
    j += ",\"saturation\":" + String(s->status.saturation);
    j += ",\"sharpness\":" + String(s->status.sharpness);
    j += ",\"awb\":" + String(s->status.awb);
    j += ",\"awb_gain\":" + String(s->status.awb_gain);
    j += ",\"wb_mode\":" + String(s->status.wb_mode);
    j += ",\"aec\":" + String(s->status.aec);
    j += ",\"aec2\":" + String(s->status.aec2);
    j += ",\"ae_level\":" + String(s->status.ae_level);
    j += ",\"aec_value\":" + String(s->status.aec_value);
    j += ",\"agc\":" + String(s->status.agc);
    j += ",\"agc_gain\":" + String(s->status.agc_gain);
    j += ",\"gainceiling\":" + String(s->status.gainceiling);
    j += ",\"bpc\":" + String(s->status.bpc);
    j += ",\"wpc\":" + String(s->status.wpc);
    j += ",\"raw_gma\":" + String(s->status.raw_gma);
    j += ",\"lenc\":" + String(s->status.lenc);
    j += ",\"hmirror\":" + String(s->status.hmirror);
    j += ",\"vflip\":" + String(s->status.vflip);
    j += ",\"dcw\":" + String(s->status.dcw);
    j += ",\"colorbar\":" + String(s->status.colorbar);
  }
  j += "}";
  return j;
}

// ==================== SPI LCD 驱动 (可选) ====================
static void xl9555_write_bit(uint8_t regOut, uint8_t regCfg, uint8_t bit, bool val) {
  uint8_t cfg = 0xFF; xl9555_read(regCfg, &cfg);
  cfg &= ~(1 << bit); xl9555_write(regCfg, cfg);
  uint8_t out = 0x00; xl9555_read(regOut, &out);
  if (val) out |= (1 << bit); else out &= ~(1 << bit);
  xl9555_write(regOut, out);
}

static inline void lcd_cs(bool on) { digitalWrite(LCD_CS_PIN, on ? LOW : HIGH); }
static inline void lcd_dc(bool data) { digitalWrite(LCD_DC_PIN, data ? HIGH : LOW); }
static void lcd_cmd(uint8_t c) { lcd_dc(false); lcd_cs(true); SPI.transfer(c); lcd_cs(false); }
static void lcd_data8(uint8_t d) { lcd_dc(true); lcd_cs(true); SPI.transfer(d); lcd_cs(false); }
static void lcd_data_buf(uint8_t *b, size_t n) { lcd_dc(true); lcd_cs(true); SPI.writeBytes(b, n); lcd_cs(false); }

static void lcd_init_panel() {
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(LCD_DC_PIN, OUTPUT);
  lcd_cs(false);
  delay(10);
  lcd_cmd(0x01); delay(150);      // SWRESET
  lcd_cmd(0x11); delay(120);      // SLPOUT
#if LCD_IS_ST7789
  lcd_cmd(0x3A); lcd_data8(0x55); // COLMOD 16bit
  lcd_cmd(0x36); lcd_data8(LCD_MADCTL);
  lcd_cmd(LCD_INVERT ? 0x21 : 0x20);
  lcd_cmd(0x13);                  // NORON
#else
  lcd_cmd(0x3A); lcd_data8(0x55);
  lcd_cmd(0x36); lcd_data8(0x28 | (LCD_MADCTL & 0xF7));
  lcd_cmd(LCD_INVERT ? 0x21 : 0x20);
#endif
  lcd_cmd(0x29); delay(50);       // DISPON
}

static void lcd_set_window(int x, int y, int w, int h) {
  lcd_cmd(0x2A);
  lcd_data8((x) >> 8); lcd_data8((x) & 0xFF);
  lcd_data8((x + w - 1) >> 8); lcd_data8((x + w - 1) & 0xFF);
  lcd_cmd(0x2B);
  lcd_data8((y) >> 8); lcd_data8((y) & 0xFF);
  lcd_data8((y + h - 1) >> 8); lcd_data8((y + h - 1) & 0xFF);
  lcd_cmd(0x2C);
}

static void lcd_fill(uint16_t color) {
  uint8_t hi = color >> 8, lo = color & 0xFF;
  uint8_t *line = (uint8_t *)malloc(LCD_W * 2);
  if (!line) return;
  for (int i = 0; i < LCD_W; i++) { line[i * 2] = hi; line[i * 2 + 1] = lo; }
  SPI.beginTransaction(SPISettings(LCD_SPI_HZ, MSBFIRST, SPI_MODE0));
  lcd_set_window(LCD_OFFSET_X, LCD_OFFSET_Y, LCD_W, LCD_H);
  lcd_dc(true); lcd_cs(true);
  for (int y = 0; y < LCD_H; y++) SPI.writeBytes(line, LCD_W * 2);
  lcd_cs(false);
  SPI.endTransaction();
  free(line);
}

static void lcd_push(uint8_t *rgb) {
  SPI.beginTransaction(SPISettings(LCD_SPI_HZ, MSBFIRST, SPI_MODE0));
  lcd_set_window(LCD_OFFSET_X, LCD_OFFSET_Y, LCD_W, LCD_H);
  lcd_data_buf(rgb, (size_t)LCD_W * LCD_H * 2);
  SPI.endTransaction();
}

// 从 JPEG 头部解析宽高 (比 fb->width 可靠)
static bool jpeg_get_size(const uint8_t *buf, size_t len, int *w, int *h) {
  size_t i = 2;
  while (i + 9 < len) {
    if (buf[i] != 0xFF) { i++; continue; }
    uint8_t m = buf[i + 1];
    if (m == 0xD8 || m == 0x01 || (m >= 0xD0 && m <= 0xD7)) { i += 2; continue; }
    uint16_t seglen = (buf[i + 2] << 8) | buf[i + 3];
    if (m >= 0xC0 && m <= 0xCF && m != 0xC4 && m != 0xC8 && m != 0xCC) {
      *h = (buf[i + 5] << 8) | buf[i + 6];
      *w = (buf[i + 7] << 8) | buf[i + 8];
      return true;
    }
    i += 2 + seglen;
  }
  return false;
}

static void lcd_task(void *arg) {
  size_t bufLen = (size_t)LCD_W * LCD_H * 2;
  uint8_t *rgb = (uint8_t *)heap_caps_malloc(bufLen, MALLOC_CAP_SPIRAM);
  if (!rgb) rgb = (uint8_t *)malloc(bufLen);
  if (!rgb) { Serial.println("[LCD] 缓冲区分配失败"); s_lcdRunning = false; vTaskDelete(NULL); return; }
  Serial.printf("[LCD] 镜像任务启动, 缓冲 %u 字节\n", (unsigned)bufLen);
  uint32_t pushed = 0, failed = 0, mism = 0;
  while (s_lcdOn) {
    if (s_recActive || s_camBusy) { vTaskDelay(50); continue; }  // 录制时 LCD 让出
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      int jw = 0, jh = 0;
      bool hasSize = jpeg_get_size(fb->buf, fb->len, &jw, &jh);
      if (hasSize && (jw != LCD_W || jh != LCD_H)) {
        // 尺寸不匹配, 强制回到设定尺寸
        sensor_t *s = esp_camera_sensor_get();
        if (s) s->set_framesize(s, LCD_FRAMESIZE);
        if (mism++ < 3) Serial.printf("[LCD] 尺寸不匹配 %dx%d (需要 %dx%d)\n", jw, jh, LCD_W, LCD_H);
        esp_camera_fb_return(fb);
        vTaskDelay(150);
        continue;
      }
      bool ok = jpg2rgb565(fb->buf, fb->len, rgb, JPG_SCALE_NONE);
      if (!ok && failed++ < 3) Serial.printf("[LCD] jpg2rgb565 失败 (len=%u)\n", (unsigned)fb->len);
      esp_camera_fb_return(fb);
      if (ok) {
#if LCD_SWAP_BYTES
        for (size_t i = 0; i < (size_t)LCD_W * LCD_H; i++) {
          uint8_t t = rgb[i * 2]; rgb[i * 2] = rgb[i * 2 + 1]; rgb[i * 2 + 1] = t;
        }
#endif
        SPI_LOCK();
        lcd_push(rgb);
        SPI_UNLOCK();
        pushed++;
        if (pushed == 1) Serial.println("[LCD] 首帧已刷屏");
      }
    }
    vTaskDelay(20);
  }
  Serial.printf("[LCD] 镜像任务结束 pushed=%u failed=%u\n", (unsigned)pushed, (unsigned)failed);
  free(rgb);
  s_lcdRunning = false;
  vTaskDelete(NULL);
}

static void lcd_set_enabled(bool on) {
  if (on == s_lcdOn) return;
  if (on) {
    if (s_recActive) return;                 // 录制中不允许开 LCD
    set_camera_param("framesize", LCD_FRAMESIZE);  // 使解码尺寸与 LCD 匹配
    delay(100);
    sensor_t *cs = esp_camera_sensor_get();
    Serial.printf("[LCD] 开启, 摄像头 framesize=%d (需 %dx%d)\n", cs ? (int)cs->status.framesize : -1, LCD_W, LCD_H);
    xl9555_write_bit(XL9555_REG_OUT1, XL9555_REG_CFG1, LCD_PWR_BIT, true);
    delay(20);
    xl9555_write_bit(XL9555_REG_OUT1, XL9555_REG_CFG1, LCD_RST_BIT, false);
    delay(20);
    xl9555_write_bit(XL9555_REG_OUT1, XL9555_REG_CFG1, LCD_RST_BIT, true);
    delay(120);
    SPI_LOCK();
    lcd_init_panel();
    // 上电测试色块: 若能看到红/绿/蓝满屏, 说明面板/接线/参数基本正确
    lcd_fill(0xF800); delay(250);
    lcd_fill(0x07E0); delay(250);
    lcd_fill(0x001F); delay(250);
    lcd_fill(0x0000); delay(150);
    SPI_UNLOCK();
    xl9555_write_bit(XL9555_REG_OUT1, XL9555_REG_CFG1, LCD_BL_BIT, true);
    s_lcdOn = true;
    s_lcdRunning = true;
    BaseType_t tr = xTaskCreatePinnedToCore(lcd_task, "lcd", 8192, NULL, 2, NULL, 1);
    Serial.printf("[LCD] 已打开 (task=%d)\n", (int)tr);
  } else {
    s_lcdOn = false;
    uint32_t t0 = millis();
    while (s_lcdRunning && millis() - t0 < 3000) delay(10);
    xl9555_write_bit(XL9555_REG_OUT1, XL9555_REG_CFG1, LCD_BL_BIT, false);
    xl9555_write_bit(XL9555_REG_OUT1, XL9555_REG_CFG1, LCD_PWR_BIT, false);
    Serial.println("[LCD] 已关闭");
  }
}

// ==================== 首页 HTML ====================
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>DNESP32S3 智能摄像头</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  :root{--bg:#0f1220;--card:rgba(255,255,255,.06);--line:rgba(255,255,255,.12);
        --acc:#4fc3f7;--acc2:#7c4dff;--ok:#00e676;--warn:#ff5252;--txt:#e8eaf6;--mut:#9aa0b4}
  body{font-family:system-ui,"Microsoft YaHei",sans-serif;color:var(--txt);min-height:100vh;
       background:radial-gradient(1200px 600px at 10% -10%,#1b2450 0%,transparent 60%),
                  radial-gradient(1000px 500px at 110% 10%,#3a1b50 0%,transparent 55%),var(--bg)}
  header{display:flex;align-items:center;justify-content:space-between;gap:12px;
         padding:16px 22px;position:sticky;top:0;z-index:10;backdrop-filter:blur(12px);
         background:rgba(15,18,32,.6);border-bottom:1px solid var(--line)}
  .brand{font-size:19px;font-weight:700;letter-spacing:.5px}
  .brand span{background:linear-gradient(90deg,var(--acc),var(--acc2));
              -webkit-background-clip:text;background-clip:text;color:transparent}
  .badges{display:flex;gap:8px;flex-wrap:wrap}
  .badge{font-size:12px;padding:4px 10px;border-radius:20px;border:1px solid var(--line);
         background:var(--card);color:var(--mut)}
  .badge.on{color:#062;background:#00e676;border-color:#00e676;font-weight:700}
  .badge.off{color:#fff;background:#c62828;border-color:#c62828;font-weight:700}
  .badge.rec{color:#fff;background:var(--warn);border-color:var(--warn);font-weight:700;
             animation:pulse 1s infinite}
  @keyframes pulse{50%{opacity:.45}}
  main{display:grid;grid-template-columns:1.6fr 1fr;gap:18px;padding:18px 22px;align-items:start}
  .card{background:var(--card);border:1px solid var(--line);border-radius:16px;
        padding:16px;box-shadow:0 10px 30px rgba(0,0,0,.25)}
  .viewer{position:relative;padding:10px}
  .viewer img{width:100%;border-radius:12px;display:block;background:#000;aspect-ratio:4/3;object-fit:contain}
  .imgwrap{position:relative;border-radius:12px;overflow:hidden}
  .pause-tip{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;
    background:rgba(0,0,0,.6);border-radius:12px;font-size:14px;color:#ffd54f;text-align:center;padding:12px}
  .actions{display:flex;gap:10px;margin-top:12px;flex-wrap:wrap}
  button{cursor:pointer;border:none;border-radius:10px;padding:10px 16px;font-size:14px;
         font-weight:600;color:#fff;background:linear-gradient(135deg,var(--acc),var(--acc2));
         transition:.2s;box-shadow:0 6px 16px rgba(79,195,247,.25)}
  button:hover{transform:translateY(-1px);filter:brightness(1.1)}
  button.ghost{background:transparent;border:1px solid var(--line);box-shadow:none;color:var(--txt)}
  button.danger{background:linear-gradient(135deg,#ff5252,#d32f2f);box-shadow:none}
  button.rec{background:linear-gradient(135deg,#ff5252,#b71c1c)}
  button:disabled{opacity:.5;cursor:not-allowed;transform:none}
  h3{font-size:15px;margin-bottom:12px;display:flex;align-items:center;justify-content:space-between}
  .ctl{margin-bottom:14px}
  .ctl label{display:flex;justify-content:space-between;font-size:13px;color:var(--mut);margin-bottom:6px}
  .ctl label b{color:var(--acc)}
  input[type=range]{width:100%;-webkit-appearance:none;height:5px;border-radius:5px;
                    background:linear-gradient(90deg,var(--acc),var(--acc2));outline:none}
  input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:16px;height:16px;
    border-radius:50%;background:#fff;border:3px solid var(--acc);cursor:pointer}
  select{width:100%;padding:8px 10px;border-radius:8px;background:#1a1f36;color:var(--txt);
         border:1px solid var(--line);font-size:13px}
  .switches{display:grid;grid-template-columns:1fr 1fr;gap:8px}
  .sw{display:flex;align-items:center;justify-content:space-between;font-size:13px;
      padding:8px 10px;border:1px solid var(--line);border-radius:9px;background:rgba(255,255,255,.03)}
  .sw input{accent-color:var(--acc);width:34px;height:18px;cursor:pointer}
  .gallery{margin:0 22px 28px}
  .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(170px,1fr));gap:14px}
  .item{border:1px solid var(--line);border-radius:12px;overflow:hidden;background:rgba(0,0,0,.25)}
  .item .thumb{width:100%;height:110px;object-fit:cover;background:#000;display:block}
  .item .vthumb{height:110px;display:flex;align-items:center;justify-content:center;font-size:34px;
                background:linear-gradient(135deg,#1b2450,#3a1b50)}
  .item .meta{padding:9px 10px;font-size:12px;color:var(--mut)}
  .item .meta .nm{color:var(--txt);font-weight:600;white-space:nowrap;overflow:hidden;
                  text-overflow:ellipsis;margin-bottom:3px}
  .item .row{display:flex;gap:6px;margin-top:8px}
  .item .row a,.item .row button{flex:1;text-align:center;font-size:12px;padding:6px 0;
     border-radius:8px;text-decoration:none}
  .item .row a{background:rgba(79,195,247,.18);color:var(--acc);border:1px solid var(--acc)}
  .item .row button{background:rgba(255,82,82,.15);color:#ff8a80;border:1px solid #ff5252}
  #toast{position:fixed;left:50%;bottom:26px;transform:translateX(-50%) translateY(120%);
    background:#1a1f36;border:1px solid var(--line);padding:12px 20px;border-radius:12px;
    font-size:14px;transition:.35s;z-index:50;box-shadow:0 12px 30px rgba(0,0,0,.5)}
  #toast.show{transform:translateX(-50%) translateY(0)}
  .empty{color:var(--mut);font-size:13px;padding:16px;text-align:center;grid-column:1/-1}
  @media(max-width:880px){main{grid-template-columns:1fr}.brand{font-size:16px}}
</style>
</head>
<body>
<header>
  <div class="brand">📷 <span>DNESP32S3 智能摄像头</span></div>
  <div class="badges">
    <span id="bWifi" class="badge">WiFi 连接中…</span>
    <span id="bSd" class="badge">SD 检测中…</span>
    <span id="bRec" class="badge rec" style="display:none">● REC <span id="recTime">00:00</span></span>
  </div>
</header>

<main>
  <section class="card viewer">
    <div class="imgwrap">
      <img id="stream" alt="视频流加载中…"/>
      <div id="pauseTip" class="pause-tip" style="display:none">正在使用摄像头拍照/录制，画面暂停…</div>
    </div>
    <div class="actions">
      <button id="btnPhoto">📸 拍照保存到 SD</button>
      <button id="btnRec" class="rec">⏺ 开始录制</button>
      <button id="btnLcd" class="ghost">🖥 LCD: 关</button>
    </div>
  </section>

  <section class="card">
    <h3>摄像头参数 <button class="ghost" onclick="resetAll()">恢复默认</button></h3>

    <div class="ctl"><label>分辨率</label>
      <select id="framesize" onchange="setP('framesize',this.value)">
        <option value="0">96x96</option><option value="1">QQVGA 160x120</option>
        <option value="2">QCIF 176x144</option><option value="3">HQVGA 240x176</option>
        <option value="4">240x240</option><option value="5">QVGA 320x240</option>
        <option value="6">CIF 400x296</option><option value="7">HVGA 480x320</option>
        <option value="8">VGA 640x480</option><option value="9">SVGA 800x600</option>
        <option value="10">XGA 1024x768</option><option value="11">HD 1280x720</option>
        <option value="12">SXGA 1280x1024</option><option value="13">UXGA 1600x1200</option>
      </select>
    </div>

    <div class="ctl"><label>画质 quality <b id="v_quality">-</b></label>
      <input type="range" id="quality" min="6" max="40" step="1" oninput="setP('quality',this.value)"></div>
    <div class="ctl"><label>亮度 brightness <b id="v_brightness">-</b></label>
      <input type="range" id="brightness" min="-2" max="2" step="1" oninput="setP('brightness',this.value)"></div>
    <div class="ctl"><label>对比度 contrast <b id="v_contrast">-</b></label>
      <input type="range" id="contrast" min="-2" max="2" step="1" oninput="setP('contrast',this.value)"></div>
    <div class="ctl"><label>饱和度 saturation <b id="v_saturation">-</b></label>
      <input type="range" id="saturation" min="-2" max="2" step="1" oninput="setP('saturation',this.value)"></div>
    <div class="ctl"><label>锐度 sharpness <b id="v_sharpness">-</b></label>
      <input type="range" id="sharpness" min="-2" max="2" step="1" oninput="setP('sharpness',this.value)"></div>
    <div class="ctl"><label>白平衡模式 wb_mode <b id="v_wb_mode">-</b></label>
      <input type="range" id="wb_mode" min="0" max="4" step="1" oninput="setP('wb_mode',this.value)"></div>
    <div class="ctl"><label>曝光等级 ae_level <b id="v_ae_level">-</b></label>
      <input type="range" id="ae_level" min="-2" max="2" step="1" oninput="setP('ae_level',this.value)"></div>
    <div class="ctl"><label>曝光值 aec_value <b id="v_aec_value">-</b></label>
      <input type="range" id="aec_value" min="0" max="1200" step="10" oninput="setP('aec_value',this.value)"></div>
    <div class="ctl"><label>增益 agc_gain <b id="v_agc_gain">-</b></label>
      <input type="range" id="agc_gain" min="0" max="30" step="1" oninput="setP('agc_gain',this.value)"></div>

    <div class="switches">
      <label class="sw">自动白平衡<input type="checkbox" id="awb" onchange="setP('awb',this.checked?1:0)"></label>
      <label class="sw">白平衡增益<input type="checkbox" id="awb_gain" onchange="setP('awb_gain',this.checked?1:0)"></label>
      <label class="sw">自动曝光<input type="checkbox" id="aec" onchange="setP('aec',this.checked?1:0)"></label>
      <label class="sw">自动增益<input type="checkbox" id="agc" onchange="setP('agc',this.checked?1:0)"></label>
      <label class="sw">水平镜像<input type="checkbox" id="hmirror" onchange="setP('hmirror',this.checked?1:0)"></label>
      <label class="sw">垂直翻转<input type="checkbox" id="vflip" onchange="setP('vflip',this.checked?1:0)"></label>
      <label class="sw">黑点校正<input type="checkbox" id="bpc" onchange="setP('bpc',this.checked?1:0)"></label>
      <label class="sw">白点校正<input type="checkbox" id="wpc" onchange="setP('wpc',this.checked?1:0)"></label>
      <label class="sw">镜头校正<input type="checkbox" id="lenc" onchange="setP('lenc',this.checked?1:0)"></label>
      <label class="sw">彩条测试<input type="checkbox" id="colorbar" onchange="setP('colorbar',this.checked?1:0)"></label>
    </div>
  </section>
</main>

<section class="card gallery">
  <h3>文件管理 <button class="ghost" onclick="loadFiles()">🔄 刷新</button></h3>
  <div id="gallery" class="grid"><div class="empty">加载中…</div></div>
</section>

<div id="toast"></div>

<script>
const $ = id => document.getElementById(id);
function toast(msg){const t=$('toast');t.textContent=msg;t.classList.add('show');
  clearTimeout(t._t);t._t=setTimeout(()=>t.classList.remove('show'),2200);}

function setP(v,val){
  fetch('/control?var='+v+'&val='+val).then(r=>r.json()).then(d=>{
    if(d.ok===false) toast('设置失败: '+v);
  }).catch(()=>toast('设置请求失败: '+v));
  const lab=$('v_'+v); if(lab) lab.textContent=val;
}
function resetAll(){
  const def={quality:12,brightness:0,contrast:0,saturation:0,sharpness:0,
             wb_mode:0,ae_level:0,aec_value:300,agc_gain:0,framesize:8};
  for(const k in def) setP(k,def[k]);
  ['awb','awb_gain','aec','agc','bpc','wpc','lenc'].forEach(k=>setP(k,1));
  ['hmirror','vflip','colorbar'].forEach(k=>setP(k,0));
  toast('已恢复默认参数'); setTimeout(loadStatus,400);
}

function loadStatus(){
  fetch('/status').then(r=>r.json()).then(s=>{
    $('bWifi').textContent='WiFi 已连接'; $('bWifi').className='badge on';
    $('bSd').textContent = s.sd ? 'SD 已挂载 (FATFS)' : 'SD 未挂载';
    $('bSd').className = 'badge ' + (s.sd?'on':'off');
    sdReady = !!s.sd;
    lcdOn = !!s.lcd;
    $('btnLcd').textContent = lcdOn ? '🖥 LCD: 开' : '🖥 LCD: 关';
    const set=(id,v)=>{const e=$(id);if(e){if(e.type==='checkbox')e.checked=!!v;else e.value=v;}
                        const l=$('v_'+id);if(l)l.textContent=v;};
    ['framesize','quality','brightness','contrast','saturation','sharpness',
     'wb_mode','ae_level','aec_value','agc_gain'].forEach(k=>set(k,s[k]));
    ['awb','awb_gain','aec','agc','hmirror','vflip','bpc','wpc','lenc','colorbar']
      .forEach(k=>set(k,s[k]));
    updateRec(s.recording);
  }).catch(()=>{ $('bWifi').textContent='离线'; $('bWifi').className='badge off'; });
}

let recTimer=null;
let sdReady=false;
let lcdOn=false;
function updateRec(on){
  $('bRec').style.display = on ? 'inline-block' : 'none';
  $('btnRec').textContent = on ? '⏹ 停止录制' : '⏺ 开始录制';
  $('btnRec').className = on ? '' : 'rec';
  $('pauseTip').style.display = busyPhoto ? 'flex' : 'none';
  if(on && !recTimer){ recTimer=setInterval(tickRec,1000); }
  if(!on && recTimer){ clearInterval(recTimer); recTimer=null; $('recTime').textContent='00:00'; }
}
let busyPhoto=false;
function tickRec(){
  fetch('/record/status').then(r=>r.json()).then(s=>{
    if(!s.recording){ updateRec(false); return; }
    const t=s.seconds, m=String(Math.floor(t/60)).padStart(2,'0'), ss=String(t%60).padStart(2,'0');
    $('recTime').textContent=m+':'+ss;
  });
}
$('btnPhoto').onclick=()=>{
  if(!sdReady){ toast('SD 卡未挂载，无法保存'); return; }
  busyPhoto=true; $('pauseTip').style.display='flex';
  toast('正在拍照…');
  fetch('/photo/save').then(r=>r.json()).then(d=>{
    if(d.ok){ toast('已保存: '+d.file); loadFiles(); }
    else toast('拍照失败: '+(d.error||''));
  }).catch(e=>toast('拍照请求失败: '+e))
    .finally(()=>{ busyPhoto=false; $('pauseTip').style.display='none'; });
};
let recReqBusy=false;
$('btnRec').onclick=()=>{
  if(!sdReady){ toast('SD 卡未挂载，无法录制'); return; }
  if(recReqBusy) return;
  const on=$('bRec').style.display!=='none';
  recReqBusy=true;
  fetch(on?'/record/stop':'/record/start').then(r=>r.json()).then(d=>{
    if(!d.ok){ toast('操作失败: '+(d.error||'')); return; }
    updateRec(!on);
    toast(on ? ('已停止录制: '+d.file) : ('开始录制: '+d.file));
    if(on) setTimeout(loadFiles, 1500);   // 等后台完成 AVI 收尾
  }).catch(e=>toast('录制请求失败: '+e))
    .finally(()=>{ recReqBusy=false; });
};

$('btnLcd').onclick=()=>{
  const want = lcdOn ? 0 : 1;
  $('btnLcd').disabled=true;
  fetch('/lcd?on='+want).then(r=>r.json()).then(d=>{
    if(!d.ok){ toast('LCD 操作失败: '+(d.error||'')); return; }
    lcdOn = !!d.lcd;
    $('btnLcd').textContent = lcdOn ? '🖥 LCD: 开' : '🖥 LCD: 关';
    toast(lcdOn ? 'LCD 已打开' : 'LCD 已关闭');
  }).catch(e=>toast('LCD 请求失败: '+e))
    .finally(()=>{ $('btnLcd').disabled=false; });
};

function loadFiles(){
  fetch('/files').then(r=>r.json()).then(list=>{
    const g=$('gallery');
    if(!list.length){
      g.innerHTML='<div class="empty">'+(sdReady?'SD 卡暂无文件':'SD 未挂载，无法读取文件')+'</div>';
      return;
    }
    g.innerHTML=list.map(f=>{
      const thumb = f.kind==='photo'
        ? '<img class="thumb" loading="lazy" src="/download?f='+encodeURIComponent(f.path)+'">'
        : '<div class="vthumb">🎬</div>';
      return '<div class="item">'+thumb+
        '<div class="meta"><div class="nm" title="'+f.name+'">'+f.name+'</div>'+
        f.sizeText+
        '<div class="row"><a href="/download?f='+encodeURIComponent(f.path)+'" download>下载</a>'+
        '<button onclick="delFile(\''+f.path+'\')">删除</button></div></div></div>';
    }).join('');
  }).catch(()=>{ $('gallery').innerHTML='<div class="empty">文件列表请求失败</div>'; });
}
function delFile(p){
  if(!confirm('确定删除 '+p+' ?')) return;
  fetch('/delete?f='+encodeURIComponent(p)).then(r=>r.json()).then(d=>{
    toast(d.ok?'已删除':'删除失败'); loadFiles();
  }).catch(e=>toast('删除请求失败: '+e));
}

window.addEventListener('load',()=>{
  $('stream').src = 'http://' + location.hostname + ':81/stream';
  loadStatus(); loadFiles(); setInterval(loadStatus,15000);
});
</script>
</body>
</html>
)HTML";

// ==================== HTTP 处理 ====================
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *req) {
  send_json(req, camera_status_json());
  return ESP_OK;
}

static esp_err_t control_handler(httpd_req_t *req) {
  char query[128], var[32], val[16];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
    send_err(req, "no query"); return ESP_OK;
  }
  if (httpd_query_key_value(query, "var", var, sizeof(var)) != ESP_OK ||
      httpd_query_key_value(query, "val", val, sizeof(val)) != ESP_OK) {
    send_err(req, "bad args"); return ESP_OK;
  }
  int res = set_camera_param(var, atoi(val));
  if (res == -1) send_err(req, "set failed");
  else send_ok(req);
  return ESP_OK;
}

static esp_err_t lcd_handler(httpd_req_t *req) {
  char query[64], val[8];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "on", val, sizeof(val)) != ESP_OK) {
    send_err(req, "bad args"); return ESP_OK;
  }
  lcd_set_enabled(atoi(val) != 0);
  String j = String("{\"ok\":true,\"lcd\":") + (s_lcdOn ? "true" : "false") + "}";
  send_json(req, j);
  return ESP_OK;
}

static esp_err_t photo_save_handler(httpd_req_t *req) {
  if (!s_sdReady) { send_err(req, "SD not mounted"); return ESP_OK; }
  s_camBusy = true;
  delay(80);
  camera_fb_t *fb = esp_camera_fb_get();
  s_camBusy = false;
  if (!fb) { send_err(req, "capture failed"); return ESP_OK; }
  SD.mkdir(PHOTO_DIR);
  char name[40];
  snprintf(name, sizeof(name), "/IMG_%04d.jpg", next_index(PHOTO_DIR, "IMG_"));
  String path = String(PHOTO_DIR) + name;
  size_t n = 0;
  bool ok = false;
  SPI_LOCK();
  File f = SD.open(path, FILE_WRITE);
  if (f) { n = f.write(fb->buf, fb->len); f.close(); ok = (n == fb->len); }
  SPI_UNLOCK();
  esp_camera_fb_return(fb);
  if (!ok) { send_err(req, "save failed"); return ESP_OK; }
  Serial.printf("[PHOTO] %s (%u bytes)\n", path.c_str(), (unsigned)n);
  String j = "{\"ok\":true,\"file\":\"" + path + "\",\"size\":" + String((uint32_t)n) + "}";
  send_json(req, j);
  return ESP_OK;
}

static esp_err_t record_start_handler(httpd_req_t *req) {
  if (!s_sdReady) { send_err(req, "SD not mounted"); return ESP_OK; }
  if (s_recActive || s_recRunning) { send_err(req, "busy, wait recording stop"); return ESP_OK; }
  SD.mkdir(VIDEO_DIR);
  char name[40];
  snprintf(name, sizeof(name), "/VID_%04d.avi", next_index(VIDEO_DIR, "VID_"));
  s_recFile = String(VIDEO_DIR) + name;
  SPI_LOCK();
  s_avi = SD.open(s_recFile, FILE_WRITE);
  bool okOpen = (bool)s_avi;
  SPI_UNLOCK();
  if (!okOpen) { send_err(req, "open failed"); return ESP_OK; }

  s_recFrames = 0; s_recMaxFrame = 0; s_headerWritten = false;
  s_frameOffsets.clear(); s_frameSizes.clear();
  s_recStartMs = millis();
  s_recActive = true; s_recRunning = true;
  xTaskCreatePinnedToCore(record_task, "rec", 8192, NULL, 2, NULL, 1);

  String j = "{\"ok\":true,\"file\":\"" + s_recFile + "\"}";
  Serial.printf("[REC] 开始录制 %s\n", s_recFile.c_str());
  send_json(req, j);
  return ESP_OK;
}

static esp_err_t record_stop_handler(httpd_req_t *req) {
  if (!s_recActive) { send_err(req, "not recording"); return ESP_OK; }
  s_recActive = false;   // 立即返回, 录制任务在后台完成 AVI 收尾
  String j = "{\"ok\":true,\"file\":\"" + s_recFile + "\",\"frames\":" +
             String(s_recFrames) + "}";
  Serial.printf("[REC] 请求停止, 后台收尾中 (frames=%u)\n", (unsigned)s_recFrames);
  send_json(req, j);
  return ESP_OK;
}

static esp_err_t record_status_handler(httpd_req_t *req) {
  String j = "{\"recording\":" + String(s_recActive ? "true" : "false") +
             ",\"frames\":" + String(s_recFrames) +
             ",\"seconds\":" + String(s_recActive ? (millis() - s_recStartMs) / 1000 : 0) + "}";
  send_json(req, j);
  return ESP_OK;
}

static esp_err_t files_handler(httpd_req_t *req) {
  if (!s_sdReady) { send_json(req, "[]"); return ESP_OK; }
  String out = "[";
  bool first = true;
  SPI_LOCK();
  append_dir_json(out, PHOTO_DIR, "photo", first);
  append_dir_json(out, VIDEO_DIR, "video", first);
  SPI_UNLOCK();
  out += "]";
  send_json(req, out);
  return ESP_OK;
}

static esp_err_t download_handler(httpd_req_t *req) {
  char query[256], raw[200], path[200];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "f", raw, sizeof(raw)) != ESP_OK) {
    send_err(req, "bad args"); return ESP_OK;
  }
  url_decode(path, raw, sizeof(path));
  File f;
  SPI_LOCK();
  f = SD.open(path, FILE_READ);
  SPI_UNLOCK();
  if (!f) { send_err(req, "not found"); return ESP_OK; }
  httpd_resp_set_type(req, mime_for(path));
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  uint8_t buf[4096];
  while (true) {
    int n;
    SPI_LOCK();
    n = f.read(buf, sizeof(buf));
    SPI_UNLOCK();
    if (n <= 0) break;
    if (httpd_resp_send_chunk(req, (const char *)buf, n) != ESP_OK) {
      SPI_LOCK(); f.close(); SPI_UNLOCK();
      return ESP_FAIL;
    }
  }
  SPI_LOCK(); f.close(); SPI_UNLOCK();
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

static esp_err_t delete_handler(httpd_req_t *req) {
  char query[256], raw[200], path[200];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "f", raw, sizeof(raw)) != ESP_OK) {
    send_err(req, "bad args"); return ESP_OK;
  }
  url_decode(path, raw, sizeof(path));
  SPI_LOCK();
  bool ok = SD.remove(path);
  SPI_UNLOCK();
  if (ok) send_ok(req); else send_err(req, "delete failed");
  return ESP_OK;
}

static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/jpeg");
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t jpg_len = 0;
  uint8_t *jpg_buf = NULL;
  char part_buf[64];
  static const char *BOUND = "\r\n--123456789000000000000987654321\r\n";
  static const char *CTYPE = "multipart/x-mixed-replace;boundary=123456789000000000000987654321";
  static const char *PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

  res = httpd_resp_set_type(req, CTYPE);
  if (res != ESP_OK) return res;

  while (true) {
    // 拍照时短暂让出摄像头; 录制时降低帧率但仍继续, 保证网页可见
    if (s_camBusy) { vTaskDelay(50); continue; }
    if (s_recActive) vTaskDelay(40);
    fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; }
    else {
      if (fb->format != PIXFORMAT_JPEG) {
        bool ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
        esp_camera_fb_return(fb); fb = NULL;
        if (!ok) res = ESP_FAIL;
      } else { jpg_len = fb->len; jpg_buf = fb->buf; }
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), PART, jpg_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, BOUND, strlen(BOUND));

    if (fb) { esp_camera_fb_return(fb); fb = NULL; jpg_buf = NULL; }
    else if (jpg_buf) { free(jpg_buf); jpg_buf = NULL; }
    if (res != ESP_OK) break;
  }
  return res;
}

static void start_camera_server() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;
  config.max_uri_handlers = 14;
  config.max_open_sockets = 5;
  config.lru_purge_enable = true;
  config.stack_size = 10240;

  httpd_uri_t uris[] = {
    { .uri = "/",              .method = HTTP_GET, .handler = index_handler,         .user_ctx = NULL },
    { .uri = "/status",        .method = HTTP_GET, .handler = status_handler,        .user_ctx = NULL },
    { .uri = "/control",       .method = HTTP_GET, .handler = control_handler,       .user_ctx = NULL },
    { .uri = "/photo/save",    .method = HTTP_GET, .handler = photo_save_handler,    .user_ctx = NULL },
    { .uri = "/record/start",  .method = HTTP_GET, .handler = record_start_handler,  .user_ctx = NULL },
    { .uri = "/record/stop",   .method = HTTP_GET, .handler = record_stop_handler,   .user_ctx = NULL },
    { .uri = "/record/status", .method = HTTP_GET, .handler = record_status_handler, .user_ctx = NULL },
    { .uri = "/files",         .method = HTTP_GET, .handler = files_handler,         .user_ctx = NULL },
    { .uri = "/download",      .method = HTTP_GET, .handler = download_handler,      .user_ctx = NULL },
    { .uri = "/delete",        .method = HTTP_GET, .handler = delete_handler,        .user_ctx = NULL },
    { .uri = "/capture",       .method = HTTP_GET, .handler = capture_handler,       .user_ctx = NULL },
    { .uri = "/lcd",           .method = HTTP_GET, .handler = lcd_handler,           .user_ctx = NULL },
  };

  if (httpd_start(&s_server, &config) == ESP_OK) {
    for (auto &u : uris) httpd_register_uri_handler(s_server, &u);
    Serial.println("[HTTP] API 服务器已启动 (端口 80)");
  } else {
    Serial.println("[HTTP] API 服务器启动失败");
  }

  // 视频流单独一个服务器(端口 81): esp_http_server 是单任务，
  // 阻塞式的 MJPEG 流会占满任务，必须与 API 分开。
  httpd_config_t scfg = HTTPD_DEFAULT_CONFIG();
  scfg.server_port = 81;
  scfg.ctrl_port = 32769;
  scfg.max_uri_handlers = 2;
  scfg.max_open_sockets = 2;
  scfg.lru_purge_enable = true;
  scfg.stack_size = 8192;
  httpd_uri_t suri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
  if (httpd_start(&s_streamServer, &scfg) == ESP_OK) {
    httpd_register_uri_handler(s_streamServer, &suri);
    Serial.println("[HTTP] 视频流服务器已启动 (端口 81)");
  } else {
    Serial.println("[HTTP] 视频流服务器启动失败");
  }
}

// ==================== 摄像头 / SD 初始化 ====================
static bool init_camera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = CAM_PIN_D0; config.pin_d1 = CAM_PIN_D1;
  config.pin_d2 = CAM_PIN_D2; config.pin_d3 = CAM_PIN_D3;
  config.pin_d4 = CAM_PIN_D4; config.pin_d5 = CAM_PIN_D5;
  config.pin_d6 = CAM_PIN_D6; config.pin_d7 = CAM_PIN_D7;
  config.pin_xclk = CAM_PIN_XCLK;
  config.pin_pclk = CAM_PIN_PCLK;
  config.pin_vsync = CAM_PIN_VSYNC;
  config.pin_href = CAM_PIN_HREF;
  config.pin_sccb_sda = CAM_PIN_SIOD;
  config.pin_sccb_scl = CAM_PIN_SIOC;
  config.pin_pwdn = CAM_PIN_PWDN;
  config.pin_reset = CAM_PIN_RESET;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count     = 1;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  if (psramFound()) {
    config.fb_count = 3;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.frame_size = FRAMESIZE_VGA;
  } else {
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] 初始化失败: 0x%x\n", err);
    return false;
  }
  sensor_t *s = esp_camera_sensor_get();
  if (s) { s->set_vflip(s, 1); s->set_hmirror(s, 0); }
  Serial.println("[CAM] 初始化成功");
  return true;
}

static void init_sd() {
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  uint32_t freqs[] = { 10000000, 4000000, 20000000 };
  for (uint32_t f : freqs) {
    if (SD.begin(SD_CS_PIN, SPI, f, "/sd")) {
      s_sdReady = true;
      SD.mkdir(PHOTO_DIR);
      SD.mkdir(VIDEO_DIR);
      Serial.printf("[SD] 挂载成功 @%uHz type=%d 容量=%lluMB 已用=%lluMB\n",
                    (unsigned)f, SD.cardType(),
                    SD.cardSize() / (1024ULL * 1024ULL),
                    SD.usedBytes() / (1024ULL * 1024ULL));
      return;
    }
    Serial.printf("[SD] %uHz 挂载失败, 重试...\n", (unsigned)f);
    SD.end();
  }
  s_sdReady = false;
  Serial.println("[SD] 挂载失败! 请检查: 1)TF 卡插好 2)格式为 FAT32 3)供电充足");
}

// ==================== setup / loop ====================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("==== DNESP32S3 智能摄像头 ====");

  camera_power_on();
  s_spiMutex = xSemaphoreCreateMutex();
  if (!init_camera()) { Serial.println("摄像头初始化失败, 停止"); return; }
  init_sd();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("连接 WiFi: %s ", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println();
  Serial.print("WiFi 已连接, IP: ");
  Serial.println(WiFi.localIP());

  start_camera_server();
  Serial.print("浏览器打开: http://");
  Serial.println(WiFi.localIP());
}

void loop() {
  delay(10000);
}
