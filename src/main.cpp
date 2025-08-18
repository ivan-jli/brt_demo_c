#include <Arduino.h>
#include <Wire.h>
#include "SparkFunLIS3DH.h"

// BRT Assignment test program
// Author: Ivan L. Ivanov
// License: MIT
// Target microcontroller: esp32-C3
// Framework: Arduino

// DEFINES
#define ACCEL_I2C_SCL 3
#define ACCEL_I2C_SDA 10
#define ACCEL_INT_GPIO_PIN 2
#define HALL_INT_GPIO_PIN 4
#define ACCEL_LED 10
#define HALL_LED 11

// Physical movements
// how much small movements, reported from the accelerometer, are needed
// in order to assume that the device is 'moving'.
#define MOVEMENT_COUNT_THRESHOLD 3
// for at least how long shall we observe small movements, in order to
// assume that the device is 'moving'.
#define MOVEMENT_DURATION_THRESHOLD_MILLISECONDS 10000

// FUNCTION DECLARATIONS
void accel_isr();
void hall_isr();
void timer_isr();

// TYPES
enum device_state_t
{
  GOING_TO_SLEEP,
  SLEEPING,
  WAKING,
  // indicating a move or a hall sensor change
  RUNNING,
  // INDICATING_ACCEL_MOVE,
  // INDICATING_HALL_CHANGE,
};

typedef struct
{
  bool active = 0; // if true, a blinking sequence is currently in progress
  u8_t led_gpio = 255;
  u8_t nbr_toggles = 0;
  u8_t current_toggle;
  // used to scale down the led toggling, making it blink slower
  u8_t divider;
  u8_t divider_current_cycle;
  // active: bool,

} led_signal_t;

typedef struct
{
  // 0 if no movement, increments with each movement detected
  u32_t counter;
  // timestamps first and last move
  unsigned long first_move_ts;
  unsigned long last_move_ts;
} movement_t;

void led_signal_init(volatile led_signal_t *ls, u8_t led_gpio, u8_t nbr_blinks, u8_t divider)
{
  ls->active = 0;
  ls->led_gpio = led_gpio;
  ls->nbr_toggles = nbr_blinks * 2;
  ls->divider = divider;
  ls->current_toggle = 0;
  ls->divider_current_cycle = 0;
}

// GLOBAL VARIABLES
LIS3DH g_lis_accel(I2C_MODE, 0x19);
static volatile device_state_t g_state;
static volatile movement_t g_mv;
static volatile led_signal_t g_accel_led_signal;
static volatile led_signal_t g_hall_led_signal;
static volatile bool g_hall_isr_flag = false;
static volatile u32_t g_accel_isr_count = 0;
static hw_timer_t *g_timer = NULL;
portMUX_TYPE g_timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_hallMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_accelMux = portMUX_INITIALIZER_UNLOCKED;
// volatile SemaphoreHandle_t g_timerSemaphore;

void setup()
{
  // Serial print
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  Serial.println("power on");

  // I2C
  Wire.begin(ACCEL_I2C_SDA, ACCEL_I2C_SCL);

  // LEDs
  led_signal_init(&g_accel_led_signal, ACCEL_LED, 1, 10);
  pinMode(g_accel_led_signal.led_gpio, OUTPUT);
  led_signal_init(&g_hall_led_signal, HALL_LED, 1, 10);
  pinMode(g_hall_led_signal.led_gpio, OUTPUT);

  // LIS3 accelerometer
  g_lis_accel.settings.accelRange = 4;
  g_lis_accel.settings.accelSampleRate = 10;
  uint8_t reg = 0;
  uint8_t *reg_ptr = &reg;
  // status_t LIS3DHCore::readRegister(uint8_t* outputPointer, uint8_t offset) {
  g_lis_accel.readRegister(reg_ptr, LIS3DH_INT1_CFG);
  // AOI-6D = ‘01’ is movement recognition
  reg &= ~(1 << 7);
  reg |= 1 << 6;
  // status_t LIS3DHCore::writeRegister(uint8_t offset, uint8_t dataToWrite) {
  g_lis_accel.writeRegister(LIS3DH_INT1_CFG, reg);

  pinMode(ACCEL_INT_GPIO_PIN, INPUT_PULLUP);

  // Hall sensor
  pinMode(HALL_INT_GPIO_PIN, INPUT_PULLUP);

  // Sleep
  // All the pins can be used to wake up the chip from Light-sleep mode, but only the pins (GPIO0 ~ GPIO5) in
  // VDD3P3_RTC domain can be used to wake up the chip from Deep-sleep mode. - p. 167 ESP32-C3 TRM (Version 1.3)
  // https://github.com/espressif/arduino-esp32/issues/7005#issuecomment-1188708571 gives example with GPIO9 though.
  esp_err_t err = esp_deep_sleep_enable_gpio_wakeup((1 << ACCEL_INT_GPIO_PIN | 1 << HALL_INT_GPIO_PIN), ESP_GPIO_WAKEUP_GPIO_LOW);
  if (err != ESP_OK)
  {
    Serial.println();
    Serial.print("esp_deep_sleep_enable_gpio_wakeup failed: ");
    Serial.print(err);
  }

  // Init variables
  g_mv.counter = 0;
  g_state = GOING_TO_SLEEP;

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(ACCEL_INT_GPIO_PIN), accel_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(HALL_INT_GPIO_PIN), hall_isr, CHANGE);
}

void loop()
{
  bool hall_isr_flag = false;
  u32_t accel_isr_count = 0;

  // }
  switch (g_state)
  {
  case GOING_TO_SLEEP:
    g_state = SLEEPING;
    esp_deep_sleep_start();
    break;
  case SLEEPING:
    delay(100); // to process any interrupts?
    g_state = WAKING;
    break;
  case WAKING:
    Serial.println("Waking up!");
    g_state = RUNNING;
    portENTER_CRITICAL(&g_timerMux);
    g_timer = timerBegin(0, 8000, true); // timer#, divider, count_up
    timerAttachInterrupt(g_timer, &timer_isr, true);
    timerAlarmWrite(g_timer, 10, true);
    timerAlarmEnable(g_timer);
    portEXIT_CRITICAL(g_timerMux);
    break;
  case RUNNING:
    // Accelerometer

    portENTER_CRITICAL(&g_accelMux);
    if (g_accel_isr_count)
    {
      g_accel_isr_count--;
      accel_isr_count++;
    }
    portEXIT_CRITICAL(&g_accelMux);
    if (accel_isr_count)
    {
      accel_isr_count--;
      portENTER_CRITICAL(&g_timerMux);
      g_mv.last_move_ts = millis();
      if (g_mv.counter == 0)
      {
        g_mv.first_move_ts = millis();
      }
      g_mv.counter++;
      portEXIT_CRITICAL(&g_timerMux);
    }

    // Hall sensor
    portENTER_CRITICAL(&g_hallMux);
    if (g_hall_isr_flag)
    {
      g_hall_isr_flag = false;
      hall_isr_flag = true;
    }
    portEXIT_CRITICAL(&g_hallMux);
    if (hall_isr_flag)
    {
      hall_isr_flag = false;
      portENTER_CRITICAL(&g_timerMux);
      if (!g_hall_led_signal.active)
      {
        g_hall_led_signal.active = true;
        g_hall_led_signal.current_toggle = 0;
        g_hall_led_signal.divider_current_cycle = 0;
      }
      portEXIT_CRITICAL(&g_timerMux);
    }
    break;
  }
  portENTER_CRITICAL(&g_timerMux);
  if (!(g_accel_led_signal.active || g_hall_led_signal.active || g_mv.counter))
  {
    g_state = GOING_TO_SLEEP;
  }
  portEXIT_CRITICAL(g_timerMux);
}

// ISR
void ARDUINO_ISR_ATTR accel_isr()
{
  portENTER_CRITICAL_ISR(&g_accelMux);
  g_accel_isr_count++;
  portEXIT_CRITICAL_ISR(&g_accelMux);
}

void ARDUINO_ISR_ATTR hall_isr()
{
  portENTER_CRITICAL_ISR(&g_hallMux);
  g_hall_isr_flag = true;
  portEXIT_CRITICAL_ISR(&g_hallMux);
}

// each 100ms when the device isn't sleeping
void ARDUINO_ISR_ATTR timer_isr()
{
  portENTER_CRITICAL_ISR(&g_timerMux);
  if (g_accel_led_signal.active)
  {
    if (g_accel_led_signal.divider_current_cycle < g_accel_led_signal.divider)
    {
      g_accel_led_signal.divider_current_cycle++;
    }
    else
    {
      // divider cycle ended
      g_accel_led_signal.divider_current_cycle = 0;
      if (g_accel_led_signal.current_toggle < g_accel_led_signal.nbr_toggles)
      {
        g_accel_led_signal.current_toggle++;
        digitalWrite(ACCEL_LED, g_accel_led_signal.current_toggle % 2);
      }
      else
      {
        // blink sequence ended
        g_accel_led_signal.active = false;
        digitalWrite(ACCEL_LED, 0); // just in case
      }
    }
  };
  if (g_hall_led_signal.active)
  {
    if (g_hall_led_signal.divider_current_cycle < g_hall_led_signal.divider)
    {
      g_hall_led_signal.divider_current_cycle++;
    }
    else
    {
      g_hall_led_signal.divider_current_cycle = 0;
      if (g_hall_led_signal.current_toggle < g_hall_led_signal.nbr_toggles)
      {
        g_hall_led_signal.current_toggle++;
        digitalWrite(HALL_LED, g_hall_led_signal.current_toggle % 2);
      }
      else
      {
        g_hall_led_signal.active = false;
        digitalWrite(HALL_LED, 0); // just in case
      }
    }
  };
  if (g_mv.counter != 0)
  {
    if (millis() - g_mv.last_move_ts > MOVEMENT_DURATION_THRESHOLD_MILLISECONDS)
    {
      // the movement is old
      g_mv.counter = 0;
    }
    else if (g_mv.last_move_ts - g_mv.first_move_ts > MOVEMENT_DURATION_THRESHOLD_MILLISECONDS)
    {
      if (g_mv.counter > MOVEMENT_COUNT_THRESHOLD)
      {
        // fresh movement for longer than 10s
        // init accel led
        g_accel_led_signal.active = true;
        g_accel_led_signal.current_toggle = 0;
        g_accel_led_signal.divider_current_cycle = 0;
      }
    }
  }
  else
  {
    g_accel_led_signal.active = false;
  }
  portEXIT_CRITICAL_ISR(&g_timerMux);
}