/**
 * @file webserver.c
 * @brief WiFi AP + HTTP server for PID tuning and blackbox download
 *
 * Creates a WiFi access point ("QuadPID") with a web interface for:
 * - Live flight data display (angles, rates, targets)
 * - PID gain adjustment and NVS save
 * - Blackbox CSV download
 * - IMU recalibration
 */

#include "webserver.h"
#include "../blackbox/blackbox.h"
#include "../config/config.h"
#include "../imu/imu.h"
#include "../rate_control/rate_control.h"
#include "../adc/adc.h"

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "esp_wifi.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// External state from main.c
extern volatile bool system_armed;

static char error_msg[128] = "";

#define WIFI_SSID "QuadPID"
#define WIFI_PASS "12345678"

static httpd_handle_t server = NULL;

// HTML page with live data display and PID tuning form
static const char *HTML_PAGE =
    "<html><head><title>PID</title>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<style>"
    ".live{background:#f0f0f0;padding:10px;margin:10px "
    "0;font-family:monospace;}"
    ".live table{border-collapse: collapse; width: 100%; max-width: 400px;}"
    ".live td, .live th{padding:4px 8px; text-align: right; border-bottom: 1px "
    "solid #ddd;}"
    ".live td:first-child{text-align: left; font-weight: bold; width: 60px;}"
    ".tgt{color:blue;} .act{color:green;} .rate{color:orange;}"
    "input[type=submit]{background:#4CAF50;color:white;padding:10px "
    "20px;border:none;border-radius:4px;cursor:pointer;margin:5px "
    "2px;font-size:14px;font-weight:bold;}"
    "input[type=submit]:hover{background:#45a049;}"
    ".btn-danger{background:#f44336 !important;}"
    ".btn-danger:hover{background:#da190b !important;}"
    ".btn-warning{background:#ff9800 !important;}"
    ".btn-warning:hover{background:#e68900 !important;}"
    ".btn-info{background:#2196F3 !important;}"
    ".btn-info:hover{background:#0b7dda !important;}"
    ".actions{background:#e8f4f8;padding:15px;margin:15px "
    "0;border-radius:6px;border:2px solid #2196F3;}"
    ".actions h3{margin:0 0 10px 0;color:#1976D2;}"
    "a{color:#2196F3;text-decoration:none;font-weight:bold;font-size:16px;}"
    "a:hover{text-decoration:underline;}"
    "</style>"
    "<script>"
    "var batSpeak=0,batTimer=0;"
    "function batWarn(mv){"
    "var now=Date.now();"
    "if(mv<=0||mv>15000)return;"
    "var v=mv/1000.0;"
    "var bs=document.getElementById('batstat');"
    "if(v<10.5){"
    "bs.innerText='CHARGE NOW!';bs.style.color='red';bs.style.fontWeight='bold';"
    "if(now-batTimer>3000){batTimer=now;var u=new SpeechSynthesisUtterance('charge the battery');u.rate=1.2;u.volume=1;speechSynthesis.speak(u);}"
    "}else if(v<11.0){"
    "bs.innerText='LOW!';bs.style.color='orangered';bs.style.fontWeight='bold';"
    "if(now-batTimer>8000){batTimer=now;var u=new SpeechSynthesisUtterance('battery low');u.rate=1.1;u.volume=1;speechSynthesis.speak(u);speechSynthesis.speak(u.cloneNode?new SpeechSynthesisUtterance('battery low'):u);}"
    "}else if(v<11.5){"
    "bs.innerText='Warning';bs.style.color='orange';bs.style.fontWeight='bold';"
    "if(now-batTimer>15000){batTimer=now;var u=new SpeechSynthesisUtterance('battery low');u.rate=1.0;u.volume=0.8;speechSynthesis.speak(u);}"
    "}else{bs.innerText='OK';bs.style.color='green';bs.style.fontWeight='normal';}}"
    "function upd(){"
    "fetch('/live').then(r=>r.json()).then(d=>{"
    "document.getElementById('ta').innerText=d.ta.toFixed(1);"
    "document.getElementById('tr').innerText=d.tr.toFixed(1);"
    "document.getElementById('tm').innerText=d.tm.toFixed(1);"
    "document.getElementById('tp').innerText=d.tp.toFixed(1);"
    "document.getElementById('rr').innerText=d.rr.toFixed(1);"
    "document.getElementById('rp').innerText=d.rp.toFixed(1);"
    "document.getElementById('roll').innerText=d.roll.toFixed(1);"
    "document.getElementById('pitch').innerText=d.pitch.toFixed(1);"
    "document.getElementById('alt').innerText=(d.alt*100).toFixed(1);"
    "document.getElementById('baro').innerText=(d.baro*100).toFixed(1);"
    "document.getElementById('laser').innerText=(d.laser*100).toFixed(1);"
    "document.getElementById('armed').innerText=d.armed?'ARMED':'DISARMED';"
    "document.getElementById('armed').style.color=d.armed?'red':'gray';"
    "document.getElementById('bat').innerText=d.bat;"
    "document.getElementById('batv').innerText=(d.bat/1000).toFixed(2)+'V';"
    "batWarn(d.bat);"
    "var errDiv=document.getElementById('errors');"
    "if(d.err && d.err.length>0){errDiv.innerHTML='<b>ERRORS:</b> "
    "'+d.err;errDiv.style.display='block';}"
    "else{errDiv.style.display='none';}"
    "}).catch(e=>{});setTimeout(upd,25);}"
    "window.onload=upd;"
    "</script>"
    "</head><body>"
    "<h2>QuadPID - Bat: <span id='bat'>%d</span> mV (<span id='batv'>--</span>) <span id='batstat' style='font-size:14px;margin-left:10px;'>--</span></h2>"
    "<h3>Status: <span id='armed' style='color:gray'>--</span> | %s</h3>"
    "<div class='live'>"
    "<b>Flight Data</b><br>"
    "<table>"
    "<tr><th></th><th class='tgt'>Tgt Angle</th><th class='tgt'>Tgt "
    "Rate</th><th class='act'>Act Rate</th><th class='act'>Angle</th></tr>"
    "<tr><td>Roll:</td><td class='tgt' id='ta'>--</td><td class='tgt' "
    "id='tr'>--</td><td class='act' "
    "id='rr'>--</td><td class='act' id='roll'>--</td></tr>"
    "<tr><td>Pitch:</td><td class='tgt' id='tm'>--</td><td class='tgt' "
    "id='tp'>--</td><td class='act' "
    "id='rp'>--</td><td class='act' id='pitch'>--</td></tr>"
    "</table>"
    "<br><b>Altitude (cm)</b><br>"
    "<table>"
    "<tr><td>Fused:</td><td id='alt'>--</td><td>(Baro+Laser)</td></tr>"
    "<tr><td>Baro:</td><td id='baro'>--</td><td>(Filtered)</td></tr>"
    "<tr><td>Laser:</td><td id='laser'>--</td><td>(< 1.0m)</td></tr>"
    "</table>"
    "<div id='errors' "
    "style='display:none;background:#ffcccc;color:red;padding:8px;margin-top:"
    "10px;border-radius:4px;font-weight:bold;'></div>"
    "</div>"
    "<form method=POST action=/s>"
    "<b>Rate Roll</b> P:<input name=rp value=%.3f size=6> "
    "I:<input name=ri value=%.3f size=6> "
    "D:<input name=rd value=%.3f size=6><br>"
    "<b>Rate Pitch</b> P:<input name=pp value=%.3f size=6> "
    "I:<input name=pi value=%.3f size=6> "
    "D:<input name=pd value=%.3f size=6><br>"
    "<b>Rate Yaw</b> P:<input name=yp value=%.3f size=6> "
    "I:<input name=yi value=%.3f size=6> "
    "D:<input name=yd value=%.3f size=6><br>"
    "<hr><b>Angle</b> P:<input name=ap value=%.2f size=6> "
    "I:<input name=ai value=%.2f size=6> "
    "(Self-Level P=Strength, I=Drift Fix)<br><br>"
    "<input type=submit value='💾 SAVE PID VALUES'></form>"
    "<form method=POST action=/r style='margin-top:5px;'>"
    "<input type=submit value='🔄 RESET TO DEFAULTS' class='btn-warning'></form>"
    "<div class='actions'>"
    "<h3>📊 Blackbox & Tools</h3>"
    "<a href=/blackbox style='display:inline-block;background:#2196F3;color:white;padding:12px "
    "24px;border-radius:4px;margin:5px;text-decoration:none;'>⬇️ Download Blackbox CSV</a><br>"
    "<form style='display:inline' method=POST action=/blackbox/clear>"
    "<input type=submit value='🗑️ Clear Blackbox' class='btn-danger'></form>"
    "<form style='display:inline' method=POST action=/calibrate>"
    "<input type=submit value='⚙️ Recalibrate IMU' class='btn-info'></form>"
    "</div>"
    "<p style='color:green'><b>%s</b></p>"
    "<p style='color:red'><b>%s</b></p>"
    "</body></html>";

static float parse_float(const char *buf, const char *key, float def) {
  char k[16];
  snprintf(k, sizeof(k), "%s=", key);
  char *p = strstr(buf, k);
  if (!p) return def;
  return strtof(p + strlen(k), NULL);
}

/* HTTP Handlers */

static esp_err_t get_handler(httpd_req_t *req) {
  char *html = malloc(6144);
  if (!html) return ESP_FAIL;

  const char *msg = "";
  const char *err_msg = "";
  char query[64];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    if (strstr(query, "msg=saved"))
      msg = "Values Inserted into PID Controller!";
    else if (strstr(query, "msg=reset"))
      msg = "Values Reset to Defaults!";
    else if (strstr(query, "msg=calibrated"))
      msg = "IMU Calibrated & Saved!";
    else if (strstr(query, "err=armed"))
      err_msg = "ERROR: Disarm first before changing settings!";
  }

  snprintf(html, 6144, HTML_PAGE,(uint16_t)(adc_get_battery_voltage_filtered()*1000.f), "Online",
           sys_cfg.roll_kp, sys_cfg.roll_ki, sys_cfg.roll_kd, sys_cfg.pitch_kp,
           sys_cfg.pitch_ki, sys_cfg.pitch_kd, sys_cfg.yaw_kp, sys_cfg.yaw_ki,
           sys_cfg.yaw_kd, sys_cfg.angle_kp, sys_cfg.angle_ki, msg, err_msg);

  httpd_resp_send(req, html, strlen(html));
  free(html);
  return ESP_OK;
}

static esp_err_t save_handler(httpd_req_t *req) {
  if (system_armed) {
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/?err=armed");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
  }

  char buf[256];
  int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (len <= 0) return ESP_FAIL;
  buf[len] = 0;

  sys_cfg.roll_kp = parse_float(buf, "rp", sys_cfg.roll_kp);
  sys_cfg.roll_ki = parse_float(buf, "ri", sys_cfg.roll_ki);
  sys_cfg.roll_kd = parse_float(buf, "rd", sys_cfg.roll_kd);
  sys_cfg.pitch_kp = parse_float(buf, "pp", sys_cfg.pitch_kp);
  sys_cfg.pitch_ki = parse_float(buf, "pi", sys_cfg.pitch_ki);
  sys_cfg.pitch_kd = parse_float(buf, "pd", sys_cfg.pitch_kd);
  sys_cfg.yaw_kp = parse_float(buf, "yp", sys_cfg.yaw_kp);
  sys_cfg.yaw_ki = parse_float(buf, "yi", sys_cfg.yaw_ki);
  sys_cfg.yaw_kd = parse_float(buf, "yd", sys_cfg.yaw_kd);
  sys_cfg.angle_kp = parse_float(buf, "ap", sys_cfg.angle_kp);
  sys_cfg.angle_ki = parse_float(buf, "ai", sys_cfg.angle_ki);

  config_save_to_nvs();
  rate_control_init();

  httpd_resp_set_status(req, "303 See Other");
  httpd_resp_set_hdr(req, "Location", "/?msg=saved");
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

static esp_err_t reset_handler(httpd_req_t *req) {
  if (system_armed) {
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/?err=armed");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
  }

  config_load_defaults();
  config_save_to_nvs();
  rate_control_init();
  httpd_resp_set_status(req, "303 See Other");
  httpd_resp_set_hdr(req, "Location", "/?msg=reset");
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

static esp_err_t blackbox_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/csv");
  httpd_resp_set_hdr(req, "Content-Disposition",
                     "attachment; filename=blackbox.csv");

  const char *header = "angle_roll,angle_pitch,"
                       "gyro_x,gyro_y,gyro_z,"
                       "pid_roll,pid_pitch,pid_yaw,"
                       "m1,m2,m3,m4,"
                       "rc_roll,rc_pitch,rc_yaw,"
                       "rc_thr,"
                       "battery_mv,"
                       "i2c_errors,accel_z_raw,"
                       "vbat_comp\n";
  httpd_resp_sendstr_chunk(req, header);

  uint16_t count = blackbox_get_count();
  char line[256];

  for (uint16_t i = 0; i < count; i++) {
    const blackbox_entry_t *e = blackbox_get_entry(i);
    if (e) {
      snprintf(line, sizeof(line),
               "%.2f,%.2f,"
               "%.2f,%.2f,%.2f,"
               "%.2f,%.2f,%.2f,"
               "%u,%u,%u,%u,"
               "%u,%u,%u,"
               "%u,"
               "%u,"
               "%lu,%.2f,%.2f\n",
               e->angle_roll, e->angle_pitch, e->gyro_x, e->gyro_y, e->gyro_z,
               e->pid_roll, e->pid_pitch, e->pid_yaw, e->motor[0], e->motor[1],
               e->motor[2], e->motor[3], 
               e->rc_roll, e->rc_pitch, e->rc_yaw,
               e->rc_throttle, e->battery_mv,
               (unsigned long)e->i2c_errors, e->accel_z_raw, e->vbat_comp);
      httpd_resp_sendstr_chunk(req, line);
    }
  }

  httpd_resp_sendstr_chunk(req, NULL);
  return ESP_OK;
}

static esp_err_t blackbox_clear_handler(httpd_req_t *req) {
  blackbox_clear();
  httpd_resp_set_status(req, "303 See Other");
  httpd_resp_set_hdr(req, "Location", "/");
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

static esp_err_t calibrate_handler(httpd_req_t *req) {
  if (system_armed) {
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/?err=armed");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
  }

  gpio_set_level(2, 1);  // LED ON during calibration
  imu_calibrate_gyro();
  imu_calibrate_accel();
  imu_calibration_save_to_nvs();
  gpio_set_level(2, 0);

  httpd_resp_set_status(req, "303 See Other");
  httpd_resp_set_hdr(req, "Location", "/?msg=calibrated");
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

// Live telemetry data
static float live_target_roll_rate = 0.0f;
static float live_target_pitch_rate = 0.0f;
static float live_target_roll_angle = 0.0f;
static float live_target_pitch_angle = 0.0f;

void webserver_set_rate_targets(float roll_rate, float pitch_rate) {
  live_target_roll_rate = roll_rate;
  live_target_pitch_rate = pitch_rate;
}

void webserver_set_angle_targets(float roll_angle, float pitch_angle) {
  live_target_roll_angle = roll_angle;
  live_target_pitch_angle = pitch_angle;
}

void webserver_set_error(const char *msg) {
  if (msg && strlen(msg) > 0) {
    snprintf(error_msg, sizeof(error_msg), "%s", msg);
  } else {
    error_msg[0] = '\0';
  }
}



static esp_err_t live_handler(httpd_req_t *req) {
  const imu_data_t *imu = imu_get_data();
  float roll_rate_act = 0.0f, pitch_rate_act = 0.0f;
  float roll_deg = 0.0f, pitch_deg = 0.0f;

  if (imu) {
    roll_rate_act = imu->gyro_x_dps;
    pitch_rate_act = imu->gyro_y_dps;
    roll_deg = imu->roll_deg;
    pitch_deg = imu->pitch_deg;
  }

  float alt_fused = 0.0f;
  float alt_baro = 0.0f;
  float alt_laser = 0.0f;
  uint16_t battery_mv =(uint16_t)(adc_get_battery_voltage_filtered()*1000.0f);

  char json[768];
  snprintf(json, sizeof(json),
           "{\"tr\":%.2f,\"tp\":%.2f,\"ta\":%.2f,\"tm\":%.2f,"
           "\"rr\":%.2f,\"rp\":%.2f,"
           "\"roll\":%.1f,\"pitch\":%.1f,"
           "\"alt\":%.2f,\"baro\":%.2f,\"laser\":%.2f,"
           "\"bat\":%d,\"armed\":%s,\"err\":\"%s\"}",
           live_target_roll_rate, live_target_pitch_rate,
           live_target_roll_angle, live_target_pitch_angle, roll_rate_act,
           pitch_rate_act, roll_deg, pitch_deg, alt_fused, alt_baro, alt_laser,
           battery_mv, system_armed ? "true" : "false", error_msg);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, json, strlen(json));
  return ESP_OK;
}

static void start_server(void) {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.core_id = 0;
  cfg.stack_size = 8192;
  cfg.max_uri_handlers = 10;

  if (httpd_start(&server, &cfg) == ESP_OK) {
    httpd_uri_t get = {.uri = "/", .method = HTTP_GET, .handler = get_handler};
    httpd_uri_t save = {.uri = "/s", .method = HTTP_POST, .handler = save_handler};
    httpd_uri_t reset = {.uri = "/r", .method = HTTP_POST, .handler = reset_handler};
    httpd_uri_t blackbox = {.uri = "/blackbox", .method = HTTP_GET, .handler = blackbox_handler};
    httpd_uri_t blackbox_clr = {.uri = "/blackbox/clear", .method = HTTP_POST, .handler = blackbox_clear_handler};
    httpd_uri_t live = {.uri = "/live", .method = HTTP_GET, .handler = live_handler};
    httpd_uri_t recal = {.uri = "/calibrate", .method = HTTP_POST, .handler = calibrate_handler};

    httpd_register_uri_handler(server, &get);
    httpd_register_uri_handler(server, &save);
    httpd_register_uri_handler(server, &reset);
    httpd_register_uri_handler(server, &blackbox);
    httpd_register_uri_handler(server, &blackbox_clr);
    httpd_register_uri_handler(server, &live);
    httpd_register_uri_handler(server, &recal);
  }
}

static void wifi_handler(void *arg, esp_event_base_t base, int32_t id,
                         void *data) {
  if (id == WIFI_EVENT_AP_START)
    start_server();
}

void webserver_init(void) {
  printf("Initializing Webserver (WiFi)...\n");
  
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                      &wifi_handler, NULL, NULL));

  wifi_config_t wcfg = {
      .ap = {
          .ssid = WIFI_SSID,
          .ssid_len = strlen(WIFI_SSID),
          .channel = 1,
          .password = WIFI_PASS,
          .max_connection = 4,
          .authmode = WIFI_AUTH_WPA_WPA2_PSK,
      },
  };

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wcfg));
  ESP_ERROR_CHECK(esp_wifi_start());
  
  // Minimum TX power to prevent heat affecting IMU
  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(8));  // 2dBm
  
  printf("WiFi Started! SSID: %s, Pass: %s\n", WIFI_SSID, WIFI_PASS);
  printf("Webserver: http://192.168.4.1\n");
}
