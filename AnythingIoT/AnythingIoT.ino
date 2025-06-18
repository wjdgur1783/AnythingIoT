#include <Arduino.h>
#include "driver/i2s.h"
#include "IoTProject25-project-1_inferencing.h"

// INMP441 마이크 핀
#define I2S_WS   47
#define I2S_SD   42
#define I2S_SCK  48

// LED 핀
#define LED_RED     15
#define LED_GREEN   16

#define SAMPLE_RATE 16000
#define I2S_BUFFER_SIZE 1024

int32_t i2s_samples[I2S_BUFFER_SIZE];
float sample_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// 상태 변수
bool led_active = false;
unsigned long led_on_start = 0;
const unsigned long LED_ON_DURATION = 10000;  // 10초 유지
const float DETECTION_THRESHOLD = 0.10f;

void setup() {
  Serial.begin(2000000);

  // I2S 설정
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = I2S_BUFFER_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  // LED 설정
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);

  Serial.println("📣 시스템 준비 완료: 전자레인지 종료음 감지 시작");
}

void loop() {
  // 10초 경과 후 LED OFF
  if (led_active && millis() - led_on_start >= LED_ON_DURATION) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    led_active = false;
  }

  // LED가 켜져 있는 동안은 감지 중단
  if (led_active) return;

  // 오디오 수집
  size_t bytes_read = 0;
  int sample_index = 0;

  while (sample_index < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
    i2s_read(I2S_NUM_0, (char *)i2s_samples, sizeof(i2s_samples), &bytes_read, portMAX_DELAY);
    int samples_read = bytes_read / sizeof(int32_t);

    for (int i = 0; i < samples_read && sample_index < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i++) {
      int32_t raw = i2s_samples[i] >> 8;
      float sample = (float)raw / (1 << 23);
      sample_buffer[sample_index++] = sample;
    }
  }

  // 신호 구조 생성
  signal_t signal;
  numpy::signal_from_buffer(sample_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

  // 추론 실행
  ei_impulse_result_t result;
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
    return;
  }

  // 결과 분석 및 LED 제어
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    ei_printf("Label %s: %.2f\n", result.classification[i].label, result.classification[i].value);

    if (strcmp(result.classification[i].label, "microwave_end") == 0 &&
        result.classification[i].value > DETECTION_THRESHOLD) {
      ei_printf("✅ 전자레인지 종료음 감지됨!\n");
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, HIGH);
      led_on_start = millis();
      led_active = true;
      break;  // 반복문 탈출
    }
  }

  delay(100);  // 간단한 추론 주기
}
