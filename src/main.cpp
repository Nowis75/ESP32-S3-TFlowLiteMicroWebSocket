// 🚀 Přepracovaný TFLite FOMO detekční server pro ESP32-S3 s MJPEG streamem a WebSocket výstupem centroidů

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "fomo_model.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"

#include "html_gui.h"

#define CAMERA_MODEL_TSIMCAM_ESP32S3
#include "camera_pins.h"

#define IMG_WIDTH 96
#define IMG_HEIGHT 96
#define GRID_SIZE 12
#define THRESHOLD 0.6f
#define TENSOR_ARENA_SIZE (350 * 1024)

const char* ssid = "embd";
const char* password = "12345678";

uint8_t* tensor_arena;
const tflite::Model* model;
tflite::MicroInterpreter* interpreter;
tflite::ErrorReporter* error_reporter;
TfLiteTensor* input_tensor;
TfLiteTensor* output_tensor;

SemaphoreHandle_t result_mutex;
httpd_handle_t camera_httpd = nullptr;
WebSocketsServer webSocket = WebSocketsServer(81);

extern const char INDEX_HTML[] PROGMEM;

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QQVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 2,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Inicializace WiFi */
void setupWiFi() {
    Serial.println("\nPřipojuji se k WiFi...");
    WiFi.begin(ssid, password);
    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED && timeout < 30) {
        delay(500);
        Serial.print(".");
        timeout++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi připojeno!");
        Serial.print("IP adresa: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nNepodařilo se připojit k WiFi!");
    }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_CONNECTED) {
        Serial.printf("WebSocket klient připojen: %u\n", num);
    } else if (type == WStype_TEXT) {
        Serial.printf("Přijatá zpráva: %s\n", payload);
    } else if (type == WStype_DISCONNECTED) {
        Serial.printf("WebSocket klient odpojen: %u\n", num);
    }
}

/* Stream handler */
esp_err_t stream_handler(httpd_req_t *req) {
    static const char *boundary = "frame";
    char part_buf[64];

    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Snímání selhalo");
            continue;
        }

        // Převod GRAYSCALE frame → JPEG
        uint8_t* jpg_buf = NULL;
        size_t jpg_len = 0;
        bool jpeg_ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len); // kvalita 80 %

        esp_camera_fb_return(fb); // frame můžeš rovnou uvolnit

        if (!jpeg_ok || jpg_buf == nullptr) {
            Serial.println("Chyba při převodu frame na JPEG");
            continue;
        }

        // Odeslání hlavičky a JPEG
        size_t hlen = snprintf(part_buf, sizeof(part_buf),
            "--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
            boundary, (unsigned int)jpg_len);

        if (httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK ||
            httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_len) != ESP_OK ||
            httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) {
            free(jpg_buf);
            break; // klient přerušil spojení
        }

        free(jpg_buf); // uvolni JPEG paměť

        vTaskDelay(pdMS_TO_TICKS(33)); // ~30 FPS
    }

    return ESP_OK;
}

void sendDetectionsWebSocket() {
    JsonDocument doc;
    JsonArray detections = doc.to<JsonArray>();
    if (xSemaphoreTake(result_mutex, portMAX_DELAY) == pdTRUE) {
        for (int y = 0; y < GRID_SIZE; y++) {
            for (int x = 0; x < GRID_SIZE; x++) {
                int i = y * GRID_SIZE + x;
                float value = (output_tensor->data.int8[i] + 128) * 0.00390625f;
                if (value > THRESHOLD) {
                    JsonObject obj = detections.add<JsonObject>();
                    obj["x"] = x;
                    obj["y"] = y;
                    obj["confidence"] = value;
                }
            }
        }
        xSemaphoreGive(result_mutex);
    }
    String output;
    serializeJson(doc, output);
    webSocket.broadcastTXT(output);
    Serial.println(output); 
}

void runInference() {
    camera_fb_t* frame = esp_camera_fb_get();
    if (!frame) return;
    uint8_t* resized = (uint8_t*) malloc(IMG_WIDTH * IMG_HEIGHT);
    if (!resized) {
        esp_camera_fb_return(frame);
        return;
    }
    if (xSemaphoreTake(result_mutex, portMAX_DELAY) == pdTRUE) {
        int idx = 0;
        for (int y = 0; y < IMG_HEIGHT; y++) {
            int src_y = y * frame->height / IMG_HEIGHT;
            for (int x = 0; x < IMG_WIDTH; x++) {
                int src_x = x * frame->width / IMG_WIDTH;
                resized[idx++] = frame->buf[src_y * frame->width + src_x];
            }
        }
        for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++) {
            input_tensor->data.int8[i] = resized[i] - 128;
        }
        unsigned long start = millis();
        interpreter->Invoke();
        unsigned long durationInv = millis() - start;
        Serial.printf("Invoke trvala %lu ms\n", durationInv);
       
        xSemaphoreGive(result_mutex);
    }
    free(resized);
    esp_camera_fb_return(frame);
    sendDetectionsWebSocket();
}

esp_err_t cors_preflight_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_send(req, NULL, 0);  // 204 No Content
    return ESP_OK;
}

httpd_uri_t cors_preflight_uri = {
    .uri = "/*",
    .method = HTTP_OPTIONS,
    .handler = cors_preflight_handler,
    .user_ctx = NULL
};

/* Spuštění WebSocket serveru */
void startWebSocketServer() {
    webSocket.begin();
    webSocket.onEvent([](uint8_t client_num, WStype_t type, uint8_t * payload, size_t length) {
        if (type == WStype_CONNECTED) {
            Serial.printf("WebSocket klient připojen: %u\n", client_num);
        } else if (type == WStype_DISCONNECTED) {
            Serial.printf("WebSocket klient odpojen: %u\n", client_num);
        }
    });
    Serial.println("WebSocket server spuštěn na portu 81");
}

/* HTTP server */
void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 8;       // Povolí více souběžných handlerů
    config.recv_wait_timeout = 10;     // Časový limit pro příjem (v sekundách)
    config.send_wait_timeout = 10;     // Časový limit pro odesílání
    config.stack_size = 8192;          // Velikost zásobníku pro každý handler
    httpd_uri_t uri_index = {
        .uri = "/", 
        .method = HTTP_GET, 
        .handler = [](httpd_req_t *req) {
        httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
        return ESP_OK;
    }};

    httpd_uri_t uri_stream = { 
        .uri = "/stream", 
        .method = HTTP_GET, 
        .handler = stream_handler 
    };
    
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &uri_index);
        httpd_register_uri_handler(camera_httpd, &uri_stream);
        httpd_register_uri_handler(camera_httpd, &cors_preflight_uri);
        Serial.println("HTTP server spuštěn na portu 80");
    } else {
        Serial.println("Chyba spuštění serveru");
    }
}

void inferenceTask(void *pvParameters) {
    while (true) {
        unsigned long start = millis();
        runInference();
        unsigned long duration = millis() - start;
        Serial.printf("Inference trvala %lu ms\n", duration);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

void webSocketTask(void *pvParameters) {
    while (true) {
        webSocket.loop();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(115200);
    setCpuFrequencyMhz(240);
    result_mutex = xSemaphoreCreateMutex();
    if (esp_camera_init(&camera_config) != ESP_OK) {
        Serial.println("Chyba inicializace kamery");
        return;
    }
    static tflite::MicroErrorReporter micro_error_reporter;
    error_reporter = &micro_error_reporter;
    model = tflite::GetModel(fomo_detector_int8_tflite);
    static tflite::AllOpsResolver resolver;
    tensor_arena = (uint8_t*) ps_malloc(TENSOR_ARENA_SIZE);
    static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, TENSOR_ARENA_SIZE, error_reporter);
    interpreter = &static_interpreter;
    interpreter->AllocateTensors();
    input_tensor = interpreter->input(0);
    output_tensor = interpreter->output(0);

    WiFi.setSleep(false);           // Zabránění úspornému režimu WiFi
    setupWiFi();
    startCameraServer();
  
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket server běží na portu 81");
    // Spuštění inference na Core 1
    xTaskCreatePinnedToCore(inferenceTask, "InferenceTask", 8192, NULL, 2, NULL, 1);
    // WebSocket na Core 0
    xTaskCreatePinnedToCore(webSocketTask, "WebSocketTask", 4096, NULL, 1, NULL, 0);
    startWebSocketServer();
}

void loop() {
    webSocket.loop();
}
