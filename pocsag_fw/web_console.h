/*
  The web console served by the board itself, over WiFi (station or the board's own access point).

    GET /        the console page (webui/pocsag_console.html, gzipped into web_page.h);
                 HTTP Basic login with the user and password set by /web user and /web pass
    GET /token   (login) a per-boot random token for the WebSocket
    GET /ws      WebSocket. The first message must be "@AUTH <token>"; after that, text lines in
                 = commands, as typed on the serial console, and everything the board prints
                 goes out to every authenticated browser. (ESP-IDF 5.x doesn't call the handler
                 at the handshake, so the token can't be checked there.)

  Built on the ESP-IDF HTTP server (esp_http_server, WebSocket support is on in the Arduino core),
  no extra library. The server runs in its own task: received lines are queued and taken from
  loop() with readLine(); sendText() queues frames as server work, so both are thread-safe.

  Plain HTTP: the password crosses the local network unencrypted. Fine at home or on the board's
  own access point, not on a public network.
*/
#pragma once
#include <Arduino.h>
#include <esp_http_server.h>
#include <esp_random.h>
#include <mbedtls/base64.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include "web_page.h"

class WebConsole {
 public:
  static const uint8_t MAX_WS = 3;             // browsers at the same time
  static const size_t  LINE   = 300;           // longest command line
  String user = "admin", pass;

  bool ready() const { return user.length() && pass.length() >= 8; }
  bool running() const { return srv_ != nullptr; }
  esp_err_t lastError() const { return err_; }       // why the last begin() failed

  bool begin() {
    if (srv_) return true;
    if (!ready()) return false;
    String up = user + ":" + pass;
    unsigned char b64[200]; size_t olen = 0;
    if (mbedtls_base64_encode(b64, sizeof b64 - 1, &olen, (const unsigned char*)up.c_str(), up.length())) return false;
    b64[olen] = 0;
    auth_ = String("Basic ") + (const char*)b64;
    char t[33];
    for (int i = 0; i < 4; i++) snprintf(t + i * 8, 9, "%08lx", (unsigned long)esp_random());
    token_ = t;
    if (!rxq_) rxq_ = xQueueCreate(8, LINE);
    for (uint8_t i = 0; i < MAX_WS; i++) fds_[i] = -1;
    self_ = this;

    httpd_config_t c = HTTPD_DEFAULT_CONFIG();
    c.max_open_sockets = 6;
    c.lru_purge_enable = true;
    c.stack_size = 6144;
    c.close_fn = onClose;
    err_ = httpd_start(&srv_, &c);
    if (err_ != ESP_OK) { srv_ = nullptr; return false; }
    httpd_uri_t root = {}; root.uri = "/"; root.method = HTTP_GET; root.handler = onRoot;
    httpd_uri_t tok = {}; tok.uri = "/token"; tok.method = HTTP_GET; tok.handler = onToken;
    httpd_uri_t ws = {}; ws.uri = "/ws"; ws.method = HTTP_GET; ws.handler = onWs; ws.is_websocket = true;
    httpd_register_uri_handler(srv_, &root);
    httpd_register_uri_handler(srv_, &tok);
    httpd_register_uri_handler(srv_, &ws);
    return true;
  }

  void end() {
    if (!srv_) return;
    httpd_stop(srv_);
    srv_ = nullptr;
    portENTER_CRITICAL(&mux_);
    for (uint8_t i = 0; i < MAX_WS; i++) fds_[i] = -1;
    portEXIT_CRITICAL(&mux_);
  }

  uint8_t clients() const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < MAX_WS; i++) if (fds_[i] >= 0) n++;
    return n;
  }

  // a command line from a browser, if one is waiting (call from loop())
  bool readLine(String& out) {
    char b[LINE];
    if (!rxq_ || xQueueReceive(rxq_, b, 0) != pdTRUE) return false;
    out = b;
    return true;
  }

  // text for every connected browser (call from loop(); normally whole lines)
  void sendText(const char* s, size_t n) {
    if (!srv_ || !n) return;
    for (uint8_t i = 0; i < MAX_WS; i++) {
      int fd = fds_[i];
      if (fd < 0) continue;
      if (pending_ > 48) { dropped_++; continue; }          // browser not keeping up
      Out* o = (Out*)malloc(sizeof(Out) + n);
      if (!o) { dropped_++; continue; }
      o->hd = srv_; o->fd = fd; o->n = n; memcpy(o->d, s, n);
      pending_++;
      if (httpd_queue_work(srv_, sendWork, o) != ESP_OK) { free(o); pending_--; dropped_++; }
    }
  }

 private:
  struct Out { httpd_handle_t hd; int fd; size_t n; char d[]; };
  static inline WebConsole* self_ = nullptr;
  httpd_handle_t srv_ = nullptr;
  QueueHandle_t  rxq_ = nullptr;
  String         auth_, token_;
  esp_err_t      err_ = ESP_OK;
  volatile int   fds_[MAX_WS] = { -1, -1, -1 };
  std::atomic<int> pending_{0};                 // frames queued, shared with the server task
  uint32_t       dropped_ = 0;
  portMUX_TYPE   mux_ = portMUX_INITIALIZER_UNLOCKED;

  bool authOk(httpd_req_t* r) {
    char h[220];
    if (httpd_req_get_hdr_value_str(r, "Authorization", h, sizeof h) != ESP_OK) return false;
    return auth_ == h;
  }
  static esp_err_t deny(httpd_req_t* r) {
    httpd_resp_set_status(r, "401 Unauthorized");
    httpd_resp_set_hdr(r, "WWW-Authenticate", "Basic realm=\"POCSAG console\", charset=\"UTF-8\"");
    httpd_resp_set_type(r, "text/plain");
    return httpd_resp_send(r, "Login required", HTTPD_RESP_USE_STRLEN);
  }
  static esp_err_t onRoot(httpd_req_t* r) {
    if (!self_->authOk(r)) return deny(r);
    httpd_resp_set_type(r, "text/html; charset=utf-8");
    httpd_resp_set_hdr(r, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(r, "Cache-Control", "no-cache");
    return httpd_resp_send(r, (const char*)WEB_PAGE_GZ, WEB_PAGE_GZ_LEN);
  }
  static esp_err_t onToken(httpd_req_t* r) {
    if (!self_->authOk(r)) return deny(r);
    httpd_resp_set_type(r, "text/plain");
    httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    return httpd_resp_send(r, self_->token_.c_str(), HTTPD_RESP_USE_STRLEN);
  }
  bool known(int fd) {
    for (uint8_t i = 0; i < MAX_WS; i++) if (fds_[i] == fd) return true;
    return false;
  }
  static esp_err_t onWs(httpd_req_t* r) {
    WebConsole* w = self_;
    int fd = httpd_req_to_sockfd(r);
    if (r->method == HTTP_GET) return ESP_OK;    // handshake (only older ESP-IDF versions call this)
    httpd_ws_frame_t f = {};
    if (httpd_ws_recv_frame(r, &f, 0) != ESP_OK) return ESP_FAIL;      // length only
    if (f.len >= LINE) {                                               // too long: read and drop
      uint8_t* junk = (uint8_t*)malloc(f.len);
      if (!junk) return ESP_FAIL;
      f.payload = junk; httpd_ws_recv_frame(r, &f, f.len); free(junk);
      return ESP_OK;
    }
    char buf[LINE];
    f.payload = (uint8_t*)buf;
    if (f.len && httpd_ws_recv_frame(r, &f, f.len) != ESP_OK) return ESP_FAIL;
    buf[f.len] = 0;
    if (f.type != HTTPD_WS_TYPE_TEXT) return ESP_OK;
    if (!w->known(fd)) {                         // first message must carry the token
      size_t n = strcspn(buf, "\r\n"); buf[n] = 0;
      if (strncmp(buf, "@AUTH ", 6) || w->token_ != buf + 6) return ESP_FAIL;   // closes the socket
      bool added = false;
      portENTER_CRITICAL(&w->mux_);
      for (uint8_t i = 0; i < MAX_WS && !added; i++) if (w->fds_[i] < 0) { w->fds_[i] = fd; added = true; }
      portEXIT_CRITICAL(&w->mux_);
      return added ? ESP_OK : ESP_FAIL;          // all browser slots taken: close
    }
    char* p = buf;                                                     // one frame may hold several lines
    while (*p) {
      char* e = strpbrk(p, "\r\n");
      if (e) *e = 0;
      if (*p) {
        char item[LINE]; strncpy(item, p, LINE - 1); item[LINE - 1] = 0;
        xQueueSend(w->rxq_, item, 0);
      }
      if (!e) break;
      p = e + 1;
    }
    return ESP_OK;
  }
  static void onClose(httpd_handle_t hd, int fd) {
    WebConsole* w = self_;
    if (w) {
      portENTER_CRITICAL(&w->mux_);
      for (uint8_t i = 0; i < MAX_WS; i++) if (w->fds_[i] == fd) w->fds_[i] = -1;
      portEXIT_CRITICAL(&w->mux_);
    }
    close(fd);                                   // with close_fn set, closing is ours
  }
  static void sendWork(void* arg) {
    Out* o = (Out*)arg;
    if (httpd_ws_get_fd_info(o->hd, o->fd) == HTTPD_WS_CLIENT_WEBSOCKET) {
      httpd_ws_frame_t f = {};
      f.type = HTTPD_WS_TYPE_TEXT;
      f.payload = (uint8_t*)o->d;
      f.len = o->n;
      f.final = true;
      httpd_ws_send_frame_async(o->hd, o->fd, &f);
    }
    if (self_) self_->pending_--;
    free(o);
  }
};
