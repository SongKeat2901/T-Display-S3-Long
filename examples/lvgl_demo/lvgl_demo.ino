#include "lvgl.h" /* https://github.com/lvgl/lvgl.git */
#include "AXS15231B.h"
#include <Arduino.h>
#include <Wire.h>
#include <ui.h>
#include <AcaiaArduinoBLE.h>
#include <Preferences.h>
#include <cstring>

// -----------------------------------------------------------------------------
// Display and Touch Configuration
// -----------------------------------------------------------------------------

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;
static lv_color_t *buf1;

constexpr int TOUCH_IICSCL = 10;
constexpr int TOUCH_IICSDA = 15;
constexpr int TOUCH_RES    = 16;

constexpr int AXS_TOUCH_ONE_POINT_LEN = 6;
constexpr int AXS_TOUCH_BUF_HEAD_LEN  = 2;

constexpr int AXS_TOUCH_GESTURE_POS = 0;
constexpr int AXS_TOUCH_POINT_NUM   = 1;
constexpr int AXS_TOUCH_EVENT_POS   = 2;
constexpr int AXS_TOUCH_X_H_POS     = 2;
constexpr int AXS_TOUCH_X_L_POS     = 3;
constexpr int AXS_TOUCH_ID_POS      = 4;
constexpr int AXS_TOUCH_Y_H_POS     = 4;
constexpr int AXS_TOUCH_Y_L_POS     = 5;
constexpr int AXS_TOUCH_WEIGHT_POS  = 6;
constexpr int AXS_TOUCH_AREA_POS    = 7;

#define AXS_GET_POINT_NUM(buf) buf[AXS_TOUCH_POINT_NUM]
#define AXS_GET_GESTURE_TYPE(buf) buf[AXS_TOUCH_GESTURE_POS]
#define AXS_GET_POINT_X(buf, point_index) (((uint16_t)(buf[AXS_TOUCH_ONE_POINT_LEN * point_index + AXS_TOUCH_X_H_POS] & 0x0F) << 8) + (uint16_t)buf[AXS_TOUCH_ONE_POINT_LEN * point_index + AXS_TOUCH_X_L_POS])
#define AXS_GET_POINT_Y(buf, point_index) (((uint16_t)(buf[AXS_TOUCH_ONE_POINT_LEN * point_index + AXS_TOUCH_Y_H_POS] & 0x0F) << 8) + (uint16_t)buf[AXS_TOUCH_ONE_POINT_LEN * point_index + AXS_TOUCH_Y_L_POS])
#define AXS_GET_POINT_EVENT(buf, point_index) (buf[AXS_TOUCH_ONE_POINT_LEN * point_index + AXS_TOUCH_EVENT_POS] >> 6)

// -----------------------------------------------------------------------------
// Brew & Application Configuration
// -----------------------------------------------------------------------------

constexpr int MAX_OFFSET          = 5;
constexpr int MIN_SHOT_DURATION_S = 5;
constexpr int MAX_SHOT_DURATION_S = 50;
constexpr int DRIP_DELAY_S        = 3;
constexpr int N                   = 10; // Samples used for trend line

constexpr int RELAY1           = 46;
constexpr int SHOT_HISTORY_CAP = 1000;

// -----------------------------------------------------------------------------
// Global State
// -----------------------------------------------------------------------------

AcaiaArduinoBLE scale;
float currentWeight       = 0.0f;
uint8_t goalWeight        = 0;
float weightOffset        = 0.0f;
bool firstBoot            = true;
int brightness            = 0;
float previousTimerValue  = 0.0f;

struct Shot
{
  float start_timestamp_s = 0.0f;
  float shotTimer         = 0.0f;
  float end_s             = 0.0f;
  float expected_end_s    = 0.0f;
  float weight[SHOT_HISTORY_CAP] = {};
  float time_s[SHOT_HISTORY_CAP] = {};
  int   datapoints = 0;
  bool  brewing    = false;
};

Shot shot;

BLEService weightService("00002a98-0000-1000-8000-00805f9b34fb");
BLEByteCharacteristic weightCharacteristic("0x2A98", BLEWrite | BLERead);

bool firstConnectionNotificationPending = true;
bool BatteryLow                         = false;

// Flushing sequence state
unsigned long startTimeFlushing     = 0;
unsigned long lastPrintTimeFlushing = 0;
const unsigned long flushDuration   = 5000; // ms
bool isFlushing                     = false;

constexpr uint32_t HUMAN_TOUCH_MIN_MS = 50;

Preferences preferences;
const char *WEIGHT_KEY     = "weight";
const char *OFFSET_KEY     = "offset";
const char *BRIGHTNESS_KEY = "brightness";

#define ENABLE_DEBUG_LOG 1

#if ENABLE_DEBUG_LOG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x) do { } while (0)
#define DEBUG_PRINTLN(x) do { } while (0)
#endif

lv_obj_t *ui_cartext = nullptr;

// -----------------------------------------------------------------------------
// Helper Utilities
// -----------------------------------------------------------------------------

static inline void setStatusLabels(const char *text)
{
  lv_label_set_text(ui_SerialLabel, text);
  lv_label_set_text(ui_SerialLabel1, text);
}

static float seconds_f()
{
  return millis() / 1000.0f;
}

static void setBrewingState(bool brewing)
{
  if (brewing)
  {
    DEBUG_PRINTLN("shot started");
    shot.start_timestamp_s = seconds_f();
    shot.shotTimer         = 0.0f;
    shot.datapoints        = 0;
    scale.startTimer();
    scale.tare();
    DEBUG_PRINTLN("Weight Timer End");
  }
  else
  {
    DEBUG_PRINTLN("ShotEnded");
    shot.end_s = seconds_f() - shot.start_timestamp_s;
    scale.stopTimer();
    digitalWrite(RELAY1, LOW);
  }
}

static void stopBrew(bool setDefaultStatus)
{
  bool wasBrewing = shot.brewing;

  shot.brewing = false;
  isFlushing   = false;

  setBrewingState(false);

  if (setDefaultStatus && wasBrewing)
  {
    setStatusLabels("Shot ended");
  }
}

static void startBrew()
{
  if (shot.brewing)
  {
    return;
  }

  isFlushing = false;
  shot.brewing = true;
  setBrewingState(true);
}

static void calculateEndTime(Shot *s)
{
  if ((s->datapoints < N) || (s->weight[s->datapoints - 1] < 10))
  {
    s->expected_end_s = MAX_SHOT_DURATION_S;
    return;
  }

  float sumXY = 0.0f;
  float sumX  = 0.0f;
  float sumY  = 0.0f;
  float sumSquaredX = 0.0f;

  for (int i = s->datapoints - N; i < s->datapoints; i++)
  {
    sumXY += s->time_s[i] * s->weight[i];
    sumX  += s->time_s[i];
    sumY  += s->weight[i];
    sumSquaredX += s->time_s[i] * s->time_s[i];
  }

  float m = (N * sumXY - sumX * sumY) / (N * sumSquaredX - (sumX * sumX));
  float meanX = sumX / N;
  float meanY = sumY / N;
  float b = meanY - m * meanX;

  s->expected_end_s = (goalWeight - weightOffset - b) / m;
}

static void enforceRelayState()
{
  bool shouldBeHigh = shot.brewing || isFlushing;
  int  actualState   = digitalRead(RELAY1);

  if (shouldBeHigh && actualState == LOW)
  {
    DEBUG_PRINTLN("Relay corrected to HIGH");
    digitalWrite(RELAY1, HIGH);
  }
  else if (!shouldBeHigh && actualState == HIGH)
  {
    DEBUG_PRINTLN("Relay was HIGH unexpectedly, forcing LOW");
    digitalWrite(RELAY1, LOW);
  }
}

// -----------------------------------------------------------------------------
// Display & Touch Callbacks
// -----------------------------------------------------------------------------

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#ifdef LCD_SPI_DMA
  char i = 0;
  while (get_lcd_spi_dma_write())
  {
    i = i >> 1;
    lcd_PushColors(0, 0, 0, 0, NULL);
  }
#endif
  lcd_PushColors(area->x1, area->y1, w, h, (uint16_t *)&color_p->full);

#ifdef LCD_SPI_DMA

#else
  lv_disp_flush_ready(disp);
#endif
}

uint8_t read_touchpad_cmd[11] = {0xb5, 0xab, 0xa5, 0x5a, 0x0, 0x0, 0x0, 0x8};
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  uint8_t buff[20] = {0};
  static bool haveLastPoint        = false;
  static lv_coord_t lastPointX     = 0;
  static lv_coord_t lastPointY     = 0;
  static constexpr uint8_t filterDepth = 2;
  static lv_coord_t historyX[filterDepth] = {0};
  static lv_coord_t historyY[filterDepth] = {0};
  static uint8_t historyCount      = 0;
  static uint8_t historyWriteIndex = 0;

  Wire.beginTransmission(0x3B);
  Wire.write(read_touchpad_cmd, 8);
  Wire.endTransmission();
  Wire.requestFrom(0x3B, 8);
  while (!Wire.available())
    ;
  Wire.readBytes(buff, 8);

  uint16_t rawX = AXS_GET_POINT_X(buff, 0);
  uint16_t rawY = AXS_GET_POINT_Y(buff, 0);
  uint16_t type = AXS_GET_GESTURE_TYPE(buff);

  if (!type && (rawX || rawY))
  {
    int32_t rotatedX = static_cast<int32_t>(EXAMPLE_LCD_V_RES - 1) - static_cast<int32_t>(rawX);
    if (rotatedX < 0)
      rotatedX = 0;
    if (rotatedX >= EXAMPLE_LCD_V_RES)
      rotatedX = EXAMPLE_LCD_V_RES - 1;

    if (rawY >= EXAMPLE_LCD_H_RES)
      rawY = EXAMPLE_LCD_H_RES - 1;

    lv_coord_t finalX = static_cast<lv_coord_t>(rawY);
    lv_coord_t finalY = static_cast<lv_coord_t>(rotatedX);
    const lv_coord_t maxX = EXAMPLE_LCD_H_RES - 1;
    const lv_coord_t maxY = EXAMPLE_LCD_V_RES - 1;

    if (haveLastPoint)
    {
      lv_coord_t deltaX = finalX - lastPointX;
      if (deltaX < 0)
        deltaX = -deltaX;
      if ((finalX <= 1 || finalX >= maxX) && deltaX > 6)
      {
        finalX = lastPointX;
      }

      lv_coord_t deltaY = finalY - lastPointY;
      if (deltaY < 0)
        deltaY = -deltaY;
      if ((finalY <= 5 || finalY >= maxY - 5) && deltaY > 20)
      {
        finalY = lastPointY;
      }

      if (deltaX > 24)
      {
        finalX = lastPointX;
      }

      if (deltaY > 32)
      {
        finalY = lastPointY;
      }
    }

    historyX[historyWriteIndex] = finalX;
    historyY[historyWriteIndex] = finalY;
    historyWriteIndex           = (historyWriteIndex + 1) % filterDepth;
    if (historyCount < filterDepth)
      historyCount++;

    lv_coord_t accumX = 0;
    lv_coord_t accumY = 0;
    for (uint8_t i = 0; i < historyCount; i++)
    {
      accumX += historyX[i];
      accumY += historyY[i];
    }

    lv_coord_t filteredX = historyCount ? (accumX / historyCount) : finalX;
    lv_coord_t filteredY = historyCount ? (accumY / historyCount) : finalY;

    lastPointX    = filteredX;
    lastPointY    = filteredY;
    haveLastPoint = true;

    data->state   = LV_INDEV_STATE_PR;
    data->point.x = lastPointX;
    data->point.y = lastPointY;

    char buf[20] = {0};
    sprintf(buf, "(%d, %d)", data->point.x, data->point.y);
    if (ui_cartext != nullptr)
      lv_label_set_text(ui_cartext, buf);
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
    haveLastPoint     = false;
    historyCount      = 0;
    historyWriteIndex = 0;
  }
}

// -----------------------------------------------------------------------------
// LVGL Event Handlers
// -----------------------------------------------------------------------------

void ui_event_FlushButton(lv_event_t *e)
{
  static uint32_t pressedAt = 0;
  lv_event_code_t event_code = lv_event_get_code(e);

  if (event_code == LV_EVENT_PRESSED)
  {
    pressedAt = millis();
    return;
  }

  if (event_code == LV_EVENT_CLICKED && (millis() - pressedAt) >= HUMAN_TOUCH_MIN_MS)
  {
    if (!isFlushing)
    {
      flushingFeature();
    }
  }
}

void ui_event_StartButton(lv_event_t *e)
{
  static uint32_t pressedAt = 0;
  lv_event_code_t event_code = lv_event_get_code(e);

  if (event_code == LV_EVENT_PRESSED)
  {
    pressedAt = millis();
    return;
  }

  if (event_code == LV_EVENT_CLICKED && (millis() - pressedAt) >= HUMAN_TOUCH_MIN_MS)
  {
    setStatusLabels("Start Button Pressed");
    startBrew();
  }
}

void ui_event_StopButton(lv_event_t *e)
{
  static uint32_t pressedAt = 0;
  lv_event_code_t event_code = lv_event_get_code(e);

  if (event_code == LV_EVENT_PRESSED)
  {
    pressedAt = millis();
    return;
  }

  if (event_code == LV_EVENT_CLICKED && (millis() - pressedAt) >= HUMAN_TOUCH_MIN_MS)
  {
    setStatusLabels("Stop Button Pressed");
    stopBrew(false);
  }
}

void ui_event_ScaleResetButton(lv_event_t *e)
{
  static uint32_t pressedAt = 0;
  lv_event_code_t event_code = lv_event_get_code(e);

  if (event_code == LV_EVENT_PRESSED)
  {
    pressedAt = millis();
    return;
  }

  if (event_code == LV_EVENT_CLICKED && (millis() - pressedAt) >= HUMAN_TOUCH_MIN_MS)
  {
    scale.tare();
    setStatusLabels("Scale Tared.");
  }
}

void ui_event_PresetWeightSlight(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_VALUE_CHANGED)
  {
    _ui_slider_set_text_value(ui_PresetWeightLabel, target, "", " g");
    int PresetWeightValue = lv_slider_get_value(target);
    saveWeight(PresetWeightValue);
    _ui_slider_set_text_value(ui_SerialLabel1, target, "Preset Weight Value Set @ ", " g");
  }
}

void ui_event_BacklightSlider(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_VALUE_CHANGED)
  {
    _ui_slider_set_text_value(ui_BacklightLabel, target, "", " %");
    _ui_slider_set_text_value(ui_SerialLabel1, target, "Backlight Value Set @ ", " %");
    int brightnessValue = lv_slider_get_value(target);
    brightness = map(brightnessValue, 0, 100, 70, 255);
    saveBrightness(brightnessValue); // Save 0-100% brightness value
    DEBUG_PRINT("Brightness value saved @ ");
    DEBUG_PRINTLN(brightnessValue);
    analogWrite(TFT_BL, brightness); // PWM based on 0-255
  }
}

void ui_event_TimerResetButton(lv_event_t *e)
{
  static uint32_t pressedAt = 0;
  lv_event_code_t event_code = lv_event_get_code(e);

  if (event_code == LV_EVENT_PRESSED)
  {
    pressedAt = millis();
    return;
  }

  if (event_code == LV_EVENT_CLICKED && (millis() - pressedAt) >= HUMAN_TOUCH_MIN_MS)
  {
    char buffer[5];
    shot.shotTimer = 0;
    dtostrf(shot.shotTimer, 5, 0, buffer);
    lv_label_set_text(ui_TimerLabel, buffer);
  }
}

// -----------------------------------------------------------------------------
// Persistence Helpers
// -----------------------------------------------------------------------------

void flushingFeature()
{
  digitalWrite(RELAY1, HIGH);   // Turn on the output pin
  startTimeFlushing = millis(); // Record the current time
  isFlushing = true;            // Set the flushing flag
  DEBUG_PRINTLN("Flushing started");
  enforceRelayState();
}

void saveBrightness(int brightness)
{
  preferences.begin("myApp", false);              // Open the preferences with a namespace and read-only flag
  preferences.putInt(BRIGHTNESS_KEY, brightness); // Write the brightness value to preferences
  preferences.end();                              // Close the preferences
}

void saveOffset(int offset)
{
  preferences.begin("myApp", false);      // Open the preferences with a namespace and read-only flag
  preferences.putInt(OFFSET_KEY, offset); // Write the brightness value to preferences
  preferences.end();                      // Close the preferences
}

void saveWeight(int weight)
{
  preferences.begin("myApp", false);      // Open the preferences with a namespace and read-only flag
  preferences.putInt(WEIGHT_KEY, weight); // Write the brightness value to preferences
  preferences.end();                      // Close the preferences
}

// -----------------------------------------------------------------------------
// Scale & Connectivity Utilities
// -----------------------------------------------------------------------------

void checkHeartBreat()
{
  if (scale.heartbeatRequired())
  {
    scale.heartbeat();
  }
}

// Call back for get battery value
const long intervalBattery = 30000; // 30 seconds
unsigned long previousMillisBattery = 0;

void getBatteryUpdate()
{
  unsigned long currentMillisBattery = millis();

  if (currentMillisBattery - previousMillisBattery >= intervalBattery)
  {
    // Save the last time you checked the battery
    previousMillisBattery = currentMillisBattery;
    // Check the battery status
    if (scale.getBattery())
    {
      DEBUG_PRINTLN("Sent Get Battery Request Completed");
    }
  }
  if (scale.updateBattery())
  {
    DEBUG_PRINT("Current Scale Battery Level @ ");
    DEBUG_PRINT(scale.batteryValue());
    DEBUG_PRINT("%");
    firstBoot = false;

    if (scale.batteryValue() < 10)
    {
      BatteryLow = true;
    }
  }
}
// end call back for get battery value

void checkScaleStatus()
{
  if (!scale.isConnected())
  {
    // Scale connected bluetooth icons
    _ui_state_modify(ui_BluetoothImage1, LV_STATE_DISABLED, _UI_MODIFY_STATE_ADD);
    _ui_state_modify(ui_BluetoothImage2, LV_STATE_DISABLED, _UI_MODIFY_STATE_ADD);
    setStatusLabels("Scale Not Connected");

    scale.init();

    currentWeight = 0;
    firstConnectionNotificationPending = true;

    // reset brew stage if lost connection
    if (shot.brewing)
    {
      stopBrew(false);
    }
  }
  else
  {
    // Scale connected bluetooth icons
    _ui_state_modify(ui_BluetoothImage1, LV_STATE_DISABLED, _UI_MODIFY_STATE_REMOVE);
    _ui_state_modify(ui_BluetoothImage2, LV_STATE_DISABLED, _UI_MODIFY_STATE_REMOVE);

    if (firstConnectionNotificationPending)
    {
      setStatusLabels("Scale Connected");
    }

    firstConnectionNotificationPending = false;
  }
}

extern uint32_t transfer_num;
extern size_t lcd_PushColors_len;

void LVGLTimerHandlerRoutine()
{
  if (transfer_num <= 0 && lcd_PushColors_len <= 0)
    lv_timer_handler();

  if (transfer_num <= 1 && lcd_PushColors_len > 0)
  {
    lcd_PushColors(0, 0, 0, 0, NULL);
  }
}

// -----------------------------------------------------------------------------
// Application Entry Points
// -----------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  delay(5000); // delay wait for the serial port to get ready

  DEBUG_PRINTLN("Serial Started");

  preferences.begin("myApp", false);                       // Open the preferences with a namespace and read-only flag
  brightness = preferences.getInt(BRIGHTNESS_KEY, 0);      // Read the brightness value from preferences
  goalWeight = preferences.getInt(WEIGHT_KEY, 0);          // Read the target weight value from preferences
  weightOffset = preferences.getInt(OFFSET_KEY, 0) / 10.0; // Read the offset value from preferences
  preferences.end();                                       // Close the preferences

  DEBUG_PRINT("Brightness read from preferences: ");
  DEBUG_PRINTLN(brightness);
  DEBUG_PRINT("Goal Weight retrieved: ");
  DEBUG_PRINTLN(goalWeight);
  DEBUG_PRINT("Offset retrieved: ");
  DEBUG_PRINTLN(weightOffset);

  if ((goalWeight < 10) || (goalWeight > 200)) // If preferences isn't initialized and has an unreasonable weight/offset, default to 36g/1.5g
  {
    goalWeight = 36;
    DEBUG_PRINTLN("Goal Weight set to: " + String(goalWeight) + " g");
  }

  if (weightOffset > MAX_OFFSET)
  {
    weightOffset = 1.5;
    DEBUG_PRINTLN("Offset set to: " + String(weightOffset) + " g");
  }

  if ((brightness < 0) || (brightness > 100)) // If preferences isn't initialized set brightness to 50%
  {
    brightness = 50;
    DEBUG_PRINTLN("Backlight set to: Default @ " + String(brightness) + " %");
  }

  // initialize the GPIO hardware
  // To add in progress
  pinMode(RELAY1, OUTPUT); // RELAY 1 Output
  digitalWrite(RELAY1, LOW);

  // initialize the Bluetooth® Low Energy hardware
  BLE.begin();
  BLE.setLocalName("shotStopper");
  BLE.setAdvertisedService(weightService);
  weightService.addCharacteristic(weightCharacteristic);
  BLE.addService(weightService);
  weightCharacteristic.writeValue(36);
  BLE.advertise();
  DEBUG_PRINTLN("Bluetooth® device active, waiting for connections...");

  pinMode(TOUCH_RES, OUTPUT);
  digitalWrite(TOUCH_RES, HIGH);
  delay(2);
  digitalWrite(TOUCH_RES, LOW);
  delay(10);
  digitalWrite(TOUCH_RES, HIGH);
  delay(2);

  Wire.begin(TOUCH_IICSDA, TOUCH_IICSCL);

  pinMode(TFT_BL, OUTPUT);    // initialized TFT Backlight Pin as output
  digitalWrite(TFT_BL, HIGH); // initialized TFT BL and set to max

  axs15231_init(); // initialized Screen

  lv_init(); // initialized LVGL
  // Display init code
  {
    size_t buffer_pixels = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES;
    size_t buffer_size = sizeof(lv_color_t) * buffer_pixels;
    buf = (lv_color_t *)ps_malloc(buffer_size);
    if (buf == NULL)
    {
      while (1)
      {
        DEBUG_PRINTLN("buf NULL");
        delay(500);
      }
    }

    buf1 = (lv_color_t *)ps_malloc(buffer_size);
    if (buf1 == NULL)
    {
      while (1)
      {
        DEBUG_PRINTLN("buf NULL");
        delay(500);
      }
    }

    lv_disp_draw_buf_init(&draw_buf, buf, buf1, buffer_pixels);
    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.sw_rotate = 1; // If you turn on software rotation, Do not update or replace LVGL
    disp_drv.rotated = LV_DISP_ROT_270;
    disp_drv.full_refresh = 1; // full_refresh must be 1
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);
  }
  ui_init(); // initialized LVGL UI intereface

  int LCDBrightness = map(brightness, 0, 100, 70, 256); // brightness value is 0~100% have to map it out to pwm value.
  analogWrite(TFT_BL, LCDBrightness);

  // initialized the Backlight Slider and Label Value
  lv_slider_set_value(ui_BacklightSlider, brightness, LV_ANIM_OFF);
  char buffer[10]; // Make sure the buffer is large enough to hold the result
  char prefixedBuffer[10];

  dtostrf(brightness, 3, 0, buffer);
  snprintf(prefixedBuffer, sizeof(prefixedBuffer), "%s %%", buffer);
  lv_label_set_text(ui_BacklightLabel, prefixedBuffer);

  // Initialized PresetWeight Slider and Label Value.
  dtostrf(goalWeight, 3, 0, buffer);
  snprintf(prefixedBuffer, sizeof(prefixedBuffer), "%s g", buffer);
  lv_label_set_text(ui_PresetWeightLabel, prefixedBuffer);

  DEBUG_PRINTLN("Setup Completed");
}

void loop()
{

  // LVGL typical timer handler routine
  LVGLTimerHandlerRoutine();
  // Check to ensure scale is connected, reconnect if failed
  checkScaleStatus();
  // Send a heartbeat message to the scale periodically to maintain connection
  checkHeartBreat();
  // Get Scale battery on first startup (got new way to update)
  if (firstBoot)
  {
    getBatteryUpdate();
  }

  // Check for flushing request
  if (isFlushing)
  {
    unsigned long currentTimeFlushing = millis();
    unsigned long elapsedTimeFlushing = currentTimeFlushing - startTimeFlushing;
    unsigned long remainingTimeFlushing = flushDuration - elapsedTimeFlushing;

    if (currentTimeFlushing - lastPrintTimeFlushing >= 1000)
    {
      lastPrintTimeFlushing = currentTimeFlushing;
      DEBUG_PRINT("Flushing... ");
      DEBUG_PRINT(remainingTimeFlushing / 1000); // Print remaining time in seconds
      DEBUG_PRINTLN(" seconds remaining");

      // Update the LVGL label
      String labelText = "Flushing... " + String(remainingTimeFlushing / 1000) + " seconds remaining";
      setStatusLabels(labelText.c_str());
    }

    // Check if x seconds have passed
    if (elapsedTimeFlushing >= flushDuration)
    {
      digitalWrite(RELAY1, LOW); // Turn off the output pin
      isFlushing = false;        // Reset the flushing flag
      DEBUG_PRINTLN("Flushing ended");
      setStatusLabels("Flushing ended");
    }
  }

  // always call newWeightAvailable to actually receive the datapoint from the scale,
  // otherwise getWeight() will return stale data
  if (scale.newWeightAvailable())
  {
    currentWeight = scale.getWeight();
    char buffer[10]; // Make sure the buffer is large enough to hold the result
    dtostrf(currentWeight, 5, 1, buffer);
    lv_label_set_text(ui_ScaleLabel, buffer);

    DEBUG_PRINT(currentWeight); // debug print weight

    // update shot trajectory
    if (shot.brewing)
    {
      if (shot.datapoints >= SHOT_HISTORY_CAP)
      {
        std::memmove(shot.time_s, shot.time_s + 1, (SHOT_HISTORY_CAP - 1) * sizeof(float));
        std::memmove(shot.weight, shot.weight + 1, (SHOT_HISTORY_CAP - 1) * sizeof(float));
        shot.datapoints = SHOT_HISTORY_CAP - 1;
      }
      shot.time_s[shot.datapoints] = seconds_f() - shot.start_timestamp_s;
      shot.weight[shot.datapoints] = currentWeight;
      shot.shotTimer = shot.time_s[shot.datapoints];
      shot.datapoints++;

      DEBUG_PRINT(" ");
      DEBUG_PRINT(shot.shotTimer);

      // update the UI timer label (maybe change to 100ms instead of 1)
      if (shot.shotTimer >= (previousTimerValue + 0.01))
      {
        dtostrf(shot.shotTimer, 5, 0, buffer);
        lv_label_set_text(ui_TimerLabel, buffer);
        previousTimerValue = shot.shotTimer; // Update previous timer value
      }

      // get the likely end time of the shot
      calculateEndTime(&shot);
      DEBUG_PRINT(" ");
      DEBUG_PRINT(shot.expected_end_s);

      // Update the LVGL label
      String labelText = "Expected end time @ " + String(shot.expected_end_s) + " s";
      setStatusLabels(labelText.c_str());
    }
    DEBUG_PRINTLN("");
  }

  // if brewing state, turn on output relay 1 (solenoid)
  if (shot.brewing == true || isFlushing)
  {
    digitalWrite(RELAY1, HIGH);
  }
  else
  {
    digitalWrite(RELAY1, LOW);
    previousTimerValue = 0;
  }

  enforceRelayState();

  // Max duration reached
  if (shot.brewing && shot.shotTimer > MAX_SHOT_DURATION_S)
  {
    DEBUG_PRINTLN("Max brew duration reached");
    setStatusLabels("Max brew duration reached");
    stopBrew(false);
  }

  // End shot
  if (shot.brewing && shot.shotTimer >= shot.expected_end_s && shot.shotTimer > MIN_SHOT_DURATION_S)
  {
    DEBUG_PRINTLN("weight achieved");
    setStatusLabels("Weight achieved");
    stopBrew(false);
  }

  // Detect error of shot
  if (shot.start_timestamp_s && shot.end_s && currentWeight >= (goalWeight - weightOffset) && seconds_f() > shot.start_timestamp_s + shot.end_s + DRIP_DELAY_S)
  {
    shot.start_timestamp_s = 0;
    shot.end_s = 0;

    DEBUG_PRINT("I detected a final weight of ");
    DEBUG_PRINT(currentWeight);
    DEBUG_PRINT("g. The goal was ");
    DEBUG_PRINT(goalWeight);
    DEBUG_PRINT("g with a negative offset of ");
    DEBUG_PRINT(weightOffset);

    if (abs(currentWeight - goalWeight + weightOffset) > MAX_OFFSET)
    {
      DEBUG_PRINT("g. Error assumed. Offset unchanged. ");
      setStatusLabels("Error assumed. Offset unchanged.");
    }
    else
    {
      DEBUG_PRINT("g. Next time I'll create an offset of ");
      weightOffset += currentWeight - goalWeight;
      DEBUG_PRINT(weightOffset);

      // Save offset
      saveOffset(weightOffset * 10);
    }
    DEBUG_PRINTLN("");
  }
}
