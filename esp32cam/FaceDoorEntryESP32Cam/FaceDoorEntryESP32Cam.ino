#include <ArduinoWebsockets.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "Arduino.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"
#include "WiFi.h"
#include <esp_now.h>
#include "esp_wifi.h"  // Cho esp_wifi_set_channel

const char* ssid = "ESP32-CAM-AP";
const char* password = "12345678";

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7
// 80:f3:da:5e:fc:94 mac của esp32 thường
// d8:13:2a:7c:4c:c4 mac của esp32 CAM
// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// MAC của ESP32 Controller (receiver for open signal)
uint8_t controller_mac[] = {0x80, 0xF3, 0xDA, 0x5E, 0xFC, 0x94};
// MAC của ESP32 CAM (this device, for reference)
uint8_t cam_mac[] = {0xd8, 0x13, 0x2a, 0x7c, 0x4c, 0xc4}; 

using namespace websockets;
WebsocketsServer socket_server;

volatile bool modeWEB = false;

dl_matrix3du_t *image_matrix = NULL;

camera_fb_t * fb = NULL;

long current_millis;
long last_detected_millis = 0;
bool retryNeeded = false;
unsigned long lastSendTime = 0;  // Thời gian gửi lần cuối
unsigned long retryInterval = 1000;  // Retry sau 1 giây nếu fail

#define relay_pin 2 // điều khiển cả door 
unsigned long door_opened_millis = 0;
long interval = 5000;           // open lock for ... milliseconds
bool face_recognised = false;

void app_facenet_main();
void app_httpserver_init();
typedef struct  {
  bool modeWEB;
} struct_message;

typedef struct  {
  bool open_door;
} open_message;

// Struct để gửi tín hiệu mở cửa
open_message door_msg;

typedef struct
{
  uint8_t *image;
  box_array_t *net_boxes;
  dl_matrix3d_t *face_id;
} http_img_process_result;

#define TAG "FACE"  // Thêm cho ESP_LOG

static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();

face_id_name_list st_face_list;
static dl_matrix3du_t *aligned_face = NULL;

httpd_handle_t camera_httpd = NULL;

typedef enum
{
  START_STREAM,
  START_DETECT,
  SHOW_FACES,
  START_RECOGNITION,
  START_ENROLL,
  ENROLL_COMPLETE,
  DELETE_ALL,
} en_fsm_state;
en_fsm_state g_state;

typedef struct
{
  char enroll_name[ENROLL_NAME_LEN];
} httpd_resp_value;

httpd_resp_value st_name;

// Hàm gửi tín hiệu mở cửa qua ESP-NOW
void sendOpenDoorSignal() {
  door_msg.open_door = true;
  esp_err_t result = esp_now_send(controller_mac, (uint8_t *) &door_msg, sizeof(door_msg));
  if (result == ESP_OK) {
    Serial.println("Send open door signal: Success");
    retryNeeded = false;
  } else {
    Serial.println("Send open door signal: Failed");
    retryNeeded = true;
    lastSendTime = millis();
  }
}

// Retry gửi nếu fail
void checkAndRetrySend() {
  if (retryNeeded && (millis() - lastSendTime > retryInterval)) {
    Serial.println("Retrying send open door signal...");
    sendOpenDoorSignal();
  }
}

// Send callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Delivery Success, door OPEN");
    retryNeeded = false;  // Thành công -> reset retry
  } else {
    Serial.println("Delivery Fail, retrying...");
    retryNeeded = true;   // Fail -> cần retry
    lastSendTime = millis();
  }
}

// Receive callback
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.print("Received from: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  if (len == sizeof(struct_message)) {
    struct_message received;
    memcpy(&received, incomingData, sizeof(received));
    modeWEB = received.modeWEB;  // Set mode từ sender
    Serial.printf("Mode updated to: %s\n", modeWEB ? "Web" : "Offline");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  digitalWrite(relay_pin, LOW);
  pinMode(relay_pin, OUTPUT);

  // cấu hình esp32cam
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  config.frame_size = FRAMESIZE_SVGA; // cấu hình ảnh lúc stream 800x600
  config.jpeg_quality = 12;
  config.fb_count = 2;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
  if (!image_matrix) {
      Serial.println("Khong cap phat duoc bo nho cho image_matrix");
  }
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA); // gán Thay đổi kích thước ảnh thực tế từ sensor FRAMESIZE_QVGA (320x240) pixel
 
  // WiFi AP mode 
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ssid, password,1,0,2);  
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  // Init ESP-NOW (moved earlier for proper order)
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);  // Đăng ký callback nhận

  // Thêm peer là controller MAC (để gửi tín hiệu mở cửa)
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, controller_mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add controller peer");
    return;
  }

  Serial.println("ESP-NOW Receiver ready. MAC: " + WiFi.macAddress());
  delay(200);  // Delay nhỏ để AP ổn định

  //Khởi tạo HTTP server và FaceNet 
  app_httpserver_init();
  app_facenet_main();

  Serial.print("AP started! SSID: ");
  Serial.println(ssid);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());  // In IP AP 
  
  socket_server.listen(82);
  Serial.println("WebSocket always listening (no restart)");

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.softAPIP());  
  Serial.println("' to connect");
}

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

httpd_uri_t index_uri = {
  .uri       = "/",
  .method    = HTTP_GET,
  .handler   = index_handler,
  .user_ctx  = NULL
};

void app_httpserver_init ()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&camera_httpd, &config) == ESP_OK)
    Serial.println("httpd_start");
  {
    httpd_register_uri_handler(camera_httpd, &index_uri);
  }
}
// lưu khuôn mặt đã đăng ký vào flash, lấy ra ở đây
void app_facenet_main()
{
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  read_face_id_from_flash_with_name(&st_face_list);
}

static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id)
{
  ESP_LOGD(TAG, "START ENROLLING");
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
  ESP_LOGD(TAG, "Face ID %s Enrollment: Sample %d",
           st_name.enroll_name,
           ENROLL_CONFIRM_TIMES - left_sample_face);
  return left_sample_face;
}

static esp_err_t send_face_list(WebsocketsClient &client)
{
  client.send("delete_faces"); // tell browser to delete all faces
  face_id_node *head = st_face_list.head;
  char add_face[64];
  for (int i = 0; i < st_face_list.count; i++) // loop current faces
  {
    sprintf(add_face, "listface:%s", head->id_name);
    client.send(add_face); //send face to browser
    head = head->next;
  }
}

static esp_err_t delete_all_faces(WebsocketsClient &client)
{
  delete_face_all_in_flash_with_name(&st_face_list);
  client.send("delete_faces");
}

void handle_message(WebsocketsClient &client, WebsocketsMessage msg)
{
  if (msg.data() == "stream") {
    g_state = START_STREAM;
    client.send("STREAMING");
  }
  if (msg.data() == "detect") {
    g_state = START_DETECT;
    client.send("DETECTING");
  }
  if (msg.data().substring(0, 8) == "capture:") {
    g_state = START_ENROLL;
    char person[FACE_ID_SAVE_NUMBER * ENROLL_NAME_LEN] = {0,};
    msg.data().substring(8).toCharArray(person, sizeof(person));
    memcpy(st_name.enroll_name, person, strlen(person) + 1);
    client.send("CAPTURING");
  }
  if (msg.data() == "recognise") {
    g_state = START_RECOGNITION;
    client.send("RECOGNISING");
  }
  if (msg.data().substring(0, 7) == "remove:") {
    char person[ENROLL_NAME_LEN * FACE_ID_SAVE_NUMBER];
    msg.data().substring(7).toCharArray(person, sizeof(person));
    delete_face_id_in_flash_with_name(&st_face_list, person);
    send_face_list(client); // reset faces in the browser
  }
  if (msg.data() == "delete_all") {
    delete_all_faces(client);
  }
}

void open_door(WebsocketsClient &client) {
  if (digitalRead(relay_pin) == LOW) {
    digitalWrite(relay_pin, HIGH); //close (energise) relay so door unlocks
    Serial.println("Door Unlocked");
    // Gửi tín hiệu mở cửa qua ESP-NOW (nếu cần controller xử lý thêm)
    sendOpenDoorSignal();
    client.send("door_open");
    door_opened_millis = millis(); // time relay closed and door opened
  }
}

void autoRecognitionOffline() {
  // Remove spam print - only print once if needed, e.g., static bool first = true; if(first) { Serial... first=false; }
  static bool first_run = true;
  if (first_run) {
    Serial.println("Entered offline mode");
    first_run = false;
  }
  
  camera_fb_t *fb_local = esp_camera_fb_get();
  if (!fb_local) return;

  fmt2rgb888(fb_local->buf, fb_local->len, fb_local->format, image_matrix->item);
  box_array_t *boxes = face_detect(image_matrix, &mtmn_config);

  if (boxes && align_face(boxes, image_matrix, aligned_face) == ESP_OK) {
    dl_matrix3d_t *face_id = get_face_id(aligned_face);

    if (st_face_list.count > 0) {
      face_id_node *match = recognize_face_with_name(&st_face_list, face_id);
      if (match) {
        Serial.printf("offline DOOR OPEN FOR: %s\n", match->id_name);
        if (digitalRead(relay_pin) == LOW) {
          digitalWrite(relay_pin, HIGH);
          // Gửi tín hiệu mở cửa qua ESP-NOW
          sendOpenDoorSignal();
          door_opened_millis = millis();
        }
      }
    }
    dl_matrix3d_free(face_id);
  }

  esp_camera_fb_return(fb_local);

  // auto close door
  if (millis() - door_opened_millis > 5000) {
    digitalWrite(relay_pin, LOW);
  }
}

void loop_with_client(WebsocketsClient &client) {
  client.onMessage(handle_message);

  dl_matrix3du_t *image_matrix_local = dl_matrix3du_alloc(1, 320, 240, 3);  // Local alloc để tránh conflict
  http_img_process_result out_res = {0};
  out_res.image = image_matrix_local->item;

  while (client.available() && modeWEB) {
    client.poll();

    // Check retry nếu cần
    checkAndRetrySend();

    if (millis() - interval > door_opened_millis) {
      digitalWrite(relay_pin, LOW);
    }

    fb = esp_camera_fb_get();
    if (g_state == START_DETECT || g_state == START_ENROLL || g_state == START_RECOGNITION)
    {
      out_res.net_boxes = NULL;
      out_res.face_id = NULL;

      fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image);

      out_res.net_boxes = face_detect(image_matrix_local, &mtmn_config);

      if (out_res.net_boxes)
      {
        if (align_face(out_res.net_boxes, image_matrix_local, aligned_face) == ESP_OK)
        {

          out_res.face_id = get_face_id(aligned_face);
          last_detected_millis = millis();
          if (g_state == START_DETECT) {
            client.send("FACE DETECTED");
          }

          if (g_state == START_ENROLL)
          {
            int left_sample_face = do_enrollment(&st_face_list, out_res.face_id);
            char enrolling_message[64];
            sprintf(enrolling_message, "SAMPLE NUMBER %d FOR %s", ENROLL_CONFIRM_TIMES - left_sample_face, st_name.enroll_name);
            client.send(enrolling_message);
            if (left_sample_face == 0)
            {
              ESP_LOGI(TAG, "Enrolled Face ID: %s", st_face_list.tail->id_name);
              g_state = START_STREAM;
              char captured_message[64];
              sprintf(captured_message, "FACE CAPTURED FOR %s", st_face_list.tail->id_name);
              client.send(captured_message);
              send_face_list(client);

            }
          }

          if (g_state == START_RECOGNITION  && (st_face_list.count > 0))
          {
            face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
            if (f)
            {
              char recognised_message[64];
              sprintf(recognised_message, "DOOR OPEN FOR %s", f->id_name);
              open_door(client);
              client.send(recognised_message);
            }
            else
            {
              client.send("FACE NOT RECOGNISED");
            }
          }
          dl_matrix3d_free(out_res.face_id);
        }

      }
      else
      {
        if (g_state != START_DETECT) {
          client.send("NO FACE DETECTED");
        }
      }

      if (g_state == START_DETECT && millis() - last_detected_millis > 500) { // Detecting but no face detected
        client.send("DETECTING");
      }

    }

    client.sendBinary((const char *)fb->buf, fb->len);

    esp_camera_fb_return(fb);
    fb = NULL;
  }
  dl_matrix3du_free(image_matrix_local);  // Free local alloc
}

void loop() {
  // Remove spam print or comment out: Serial.printf("Loop tick, modeWEB: %s\n", modeWEB ? "Web" : "Offline");

  // Check retry gửi ở mọi mode
  checkAndRetrySend();

  if (modeWEB) {  
    // Web mode: Luôn poll/accept (server luôn on)
    socket_server.poll();   // Xử lý WS không block
    if (socket_server.available()) {  // Tránh block accept
      auto client = socket_server.accept();
      if (client.available()) { 
        Serial.println("client connected"); 
        send_face_list(client); 
        client.send("STREAMING");
        loop_with_client(client); // Chạy chế độ Web điều khiển 
      } 
    }
  } else {
    autoRecognitionOffline();
  }
  delay(100);  // Nhẹ để responsive
  yield();
}