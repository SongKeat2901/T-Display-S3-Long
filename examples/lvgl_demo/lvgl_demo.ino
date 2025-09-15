#include "lvgl.h" /* https://github.com/lvgl/lvgl.git */
#include <driver/timer.h>
#include "AXS15231B.h"
#include <Arduino.h>
#include <Wire.h>
#include <ui.h>
#include <AcaiaArduinoBLE.h>
#include <EEPROM.h>
#include <Preferences.h>

//================================
// If you turn on software rotation(disp_drv.sw_rotate = 1), Do not update or replace LVGL.
// disp_drv.full_refresh must be 1
//================================

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;
static lv_color_t *buf1;

uint8_t ALS_ADDRESS = 0x3B;
#define TOUCH_IICSCL 10
#define TOUCH_IICSDA 15
#define TOUCH_INT 11
#define TOUCH_RES 16

#define AXS_TOUCH_ONE_POINT_LEN 6
#define AXS_TOUCH_BUF_HEAD_LEN 2

#define AXS_TOUCH_GESTURE_POS 0
#define AXS_TOUCH_POINT_NUM 1
#define AXS_TOUCH_EVENT_POS 2
#define AXS_TOUCH_X_H_POS 2
#define AXS_TOUCH_X_L_POS 3
#define AXS_TOUCH_ID_POS 4
#define AXS_TOUCH_Y_H_POS 4
#define AXS_TOUCH_Y_L_POS 5
#define AXS_TOUCH_WEIGHT_POS 6
#define AXS_TOUCH_AREA_POS 7

#define AXS_GET_POINT_NUM(buf) buf[AXS_TOUCH_POINT_NUM]
#define AXS_GET_GESTURE_TYPE(buf) buf[AXS_TOUCH_GESTURE_POS]
#define AXS_GET_POINT_X(buf, point_index) (((uint16_t)(buf[AXS_TOUCH_ONE_POINT_LEN * point_index + AXS_TOUCH_X_H_POS] & 0x0F) << 8) + (uint16_t)buf[AXS_TOUCH_ONE_POINT_LEN * point_index + AXS_TOUCH_X_L_POS])
#define AXS_GET_POINT_Y(buf, point_index) (((uint16_t)(buf[AXS_TOUCH_ONE_POINT_LEN * point_index + AXS_TOUCH_Y_H_POS] & 0x0F) << 8) + (uint16_t)buf[AXS_TOUCH_ONE_POINT_LEN * point_index + AXS_TOUCH_Y_L_POS])
#define AXS_GET_POINT_EVENT(buf, point_index) (buf[AXS_TOUCH_ONE_POINT_LEN * point_index + AXS_TOUCH_EVENT_POS] >> 6)

// Acaia Setup
#define MAX_OFFSET 5           // In case an error in brewing occured
#define MIN_SHOT_DURATION_S 3  // Useful for flushing the group.
                               //  This ensure that the system will ignore
                               //  "shots" that last less than this duration
#define MAX_SHOT_DURATION_S 50 // Primarily useful for latching switches, since user
                               //  looses control of the paddle once the system
                               //  latches.
#define BUTTON_READ_PERIOD_MS 30
#define DRIP_DELAY_S 3 // Time after the shot ended to measure the final weight

#define EEPROM_SIZE 2 // This is 1-Byte
#define WEIGHT_ADDR 0 // Use the first byte of EEPROM to store the goal weight
#define OFFSET_ADDR 1
#define BACKLIGHT_ADDR 2

#define N 10 // Number of datapoints used to calculate trend line

// User defined***
#define MOMENTARY true   // Define brew switch style.
                         // True for momentary switches such as GS3 AV, Silvia Pro
                         // false for latching switches such as Linea Mini/Micra
#define REEDSWITCH false // Set to true if the brew state is being determined
                         //  by a reed switch attached to the brew solenoid

// Output Relay Control Pin
#define RELAY1 46

#define TIMER_INTERVAL_US 10000 // Adjust this value as needed

//***************

AcaiaArduinoBLE scale;
float currentWeight = 0;
uint8_t goalWeight = 0; // Goal Weight to be read from flash memory esp32
float weightOffset = 0;
float error = 0;
int buttonArr[4]; // last 4 readings of the button
bool FirstBoot = true;
int out = 11;
bool buttonPressed = false; // physical status of button
bool buttonLatched = false; // electrical status of button
unsigned long lastButtonRead_ms = 0;
int newButtonState = 0;
int brightness;
float previousTimerValue = 0;

struct Shot
{
  float start_timestamp_s; // Relative to runtime
  float shotTimer;         // Reset when the final drip measurement is made
  float end_s;             // Number of seconds after the shot started
  float expected_end_s;    // Estimated duration of the shot
  float weight[1000];      // A scatter plot of the weight measurements, along with time_s[]
  float time_s[1000];      // Number of seconds after the shot starte
  int datapoints;          // Number of datapoitns in the scatter plot
  bool brewing;            // True when actively brewing, otherwise false
};

// Initialize shot
Shot shot = {0, 0, 0, 0, {}, {}, 0, false};

// BLE peripheral device
BLEService weightService("00002a98-0000-1000-8000-00805f9b34fb"); // create service
BLEByteCharacteristic weightCharacteristic("0x2A98", BLEWrite | BLERead);

bool firstime = true;
bool BatteryLow = false;
float seconds_f();
void setBrewingState(bool);
void calculateEndTime(Shot *s);
void setBrewingState(bool brewing);


// flushing 
unsigned long startTimeFlushing = 0;   // Variable to store the start time
bool isFlushing = false;
const unsigned long flushDuration = 5000; // Duration of flushing in milliseconds (10 seconds)
unsigned long lastPrintTimeFlushing = 0; // To keep track of the last time we printed the countdown


// memory
Preferences preferences;
const char *WEIGHT_KEY = "weight";
const char *OFFSET_KEY = "offset";
const char *BRIGHTNESS_KEY = "brightness";

lv_obj_t *ui_cartext = NULL;

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

  Wire.beginTransmission(0x3B);
  Wire.write(read_touchpad_cmd, 8);
  Wire.endTransmission();
  Wire.requestFrom(0x3B, 8);
  while (!Wire.available())
    ;
  Wire.readBytes(buff, 8);

  uint16_t pointX;
  uint16_t pointY;
  uint16_t type = 0;

  type = AXS_GET_GESTURE_TYPE(buff);
  pointX = AXS_GET_POINT_X(buff, 0);
  pointY = AXS_GET_POINT_Y(buff, 0);

  if (!type && (pointX || pointY))
  {
    pointX = (640 - pointX);
    if (pointX > 640)
      pointX = 640;
    if (pointY > 180)
      pointY = 180;
    data->state = LV_INDEV_STATE_PR;
    data->point.x = pointY;
    data->point.y = pointX;

    char buf[20] = {0};
    sprintf(buf, "(%d, %d)", data->point.x, data->point.y);
    if (ui_cartext != NULL)
      lv_label_set_text(ui_cartext, buf);
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}

void ui_event_FlushButton(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_CLICKED)
  {
    if (!isFlushing)
    {
      flushingFeature();
      lv_label_set_text(ui_SerialLabel, "10 Seconds Flushing Started");
    }
  }
}

void ui_event_StartButton(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_CLICKED)
  {
    buttonPressed = true;
    newButtonState = 1;
    lv_label_set_text(ui_SerialLabel, "Start Button Pressed");
  }
}

void ui_event_StopButton(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_CLICKED)
  {
    buttonPressed = false;
    newButtonState = 0;
    shot.brewing = false;
    isFlushing = false;  
    setBrewingState(shot.brewing);
    digitalWrite(RELAY1, LOW);
    lv_label_set_text(ui_SerialLabel, "Stop Button Pressed");
  }
}

void ui_event_ScaleResetButton(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_CLICKED)
  {
    //scale.tare(); if tare twice in a row, for old acaia lunar, the controller will freeze - reason unknow
    scale.tare();
    lv_label_set_text(ui_SerialLabel, "Scale Tared.");
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
    brightness = map(brightnessValue, 0, 100, 70, 256);
    saveBrightness(brightnessValue); // Save 0-100% brightness value
    Serial.print("Brightness value saved @ ");
    Serial.println(brightnessValue);
    analogWrite(TFT_BL, brightness); // PWM based on 0-255
  }
}

void ui_event_TimerResetButton(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_CLICKED)
  {
    char buffer[5];
    shot.shotTimer = 0;
    dtostrf(shot.shotTimer, 5, 0, buffer);
    lv_label_set_text(ui_TimerLabel, buffer);
  }
}

void flushingFeature ()
{
  digitalWrite(RELAY1, HIGH);  // Turn on the output pin
  startTimeFlushing = millis();           // Record the current time
  isFlushing = true;              // Set the flushing flag
  Serial.println("Flushing started");
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

void checkHeartBreat()
{
  if (scale.heartbeatRequired())
  {
    scale.heartbeat();
  }
}


// Call back for get battery value
const long intervalBattery = 30000;// 30 seconds
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
      Serial.println("Sent Get Battery Request Completed");
    }
  }
  if (scale.updateBattery())
  { 
    Serial.print("Current Scale Battery Level @ ");
    Serial.print(scale.batteryValue());
    Serial.print("%");
    FirstBoot = false;

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
    lv_label_set_text(ui_SerialLabel, "Scale Not Connected");
    lv_label_set_text(ui_SerialLabel1, "Scale Not Connected");

    scale.init();

    currentWeight = 0;
    firstime = true;

    // reset brew stage if lost connection
    if (shot.brewing)
    {
      setBrewingState(false);
    }
  }
  else
  {
    // Scale connected bluetooth icons
    _ui_state_modify(ui_BluetoothImage1, LV_STATE_DISABLED, _UI_MODIFY_STATE_REMOVE);
    _ui_state_modify(ui_BluetoothImage2, LV_STATE_DISABLED, _UI_MODIFY_STATE_REMOVE);

    if (firstime == true)
    {
      lv_label_set_text(ui_SerialLabel, "Scale Connected");
      lv_label_set_text(ui_SerialLabel1, "Scale Connected");
    }

    firstime = false;
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

void setup()
{
  Serial.begin(115200);
  delay(5000); // delay wait for the serial port to get ready

  Serial.println("Serial Started");

  preferences.begin("myApp", false);                       // Open the preferences with a namespace and read-only flag
  brightness = preferences.getInt(BRIGHTNESS_KEY, 0);      // Read the brightness value from preferences
  goalWeight = preferences.getInt(WEIGHT_KEY, 0);          // Read the target weight value from preferences
  weightOffset = preferences.getInt(OFFSET_KEY, 0) / 10.0; // Read the offset value from preferences
  preferences.end();                                       // Close the preferences

  Serial.print("Brightness read from preferences: ");
  Serial.println(brightness);
  Serial.print("Goal Weight retrieved: ");
  Serial.println(goalWeight);
  Serial.print("Offset retrieved: ");
  Serial.println(weightOffset);

  if ((goalWeight < 10) || (goalWeight > 200)) // If preferences isn't initialized and has an unreasonable weight/offset, default to 36g/1.5g
  {
    goalWeight = 36;
    Serial.println("Goal Weight set to: " + String(goalWeight) + " g");
  }

  if (weightOffset > MAX_OFFSET)
  {
    weightOffset = 1.5;
    Serial.println("Offset set to: " + String(weightOffset) + " g");
  }

  if ((brightness < 0) || (brightness > 100)) // If preferences isn't initialized set brightness to 50%
  {
    brightness = 50;
    Serial.println("Backlight set to: Default @ " + String(brightness) + " %");
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
  Serial.println("Bluetooth® device active, waiting for connections...");

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
    size_t buffer_size = sizeof(lv_color_t) * EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES;
    buf = (lv_color_t *)ps_malloc(buffer_size);
    if (buf == NULL)
    {
      while (1)
      {
        Serial.println("buf NULL");
        delay(500);
      }
    }

    buf1 = (lv_color_t *)ps_malloc(buffer_size);
    if (buf1 == NULL)
    {
      while (1)
      {
        Serial.println("buf NULL");
        delay(500);
      }
    }

    lv_disp_draw_buf_init(&draw_buf, buf, buf1, buffer_size);
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

  Serial.println("Setup Completed");
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
  if (FirstBoot == true)
  {
    getBatteryUpdate();
  }

  // Check for flushing request
 if (isFlushing) 
 {
    unsigned long currentTimeFlushing = millis();
    unsigned long elapsedTimeFlushing = currentTimeFlushing - startTimeFlushing;
    unsigned long remainingTimeFlushing = flushDuration - elapsedTimeFlushing;

    if (currentTimeFlushing - lastPrintTimeFlushing >= 1000) {
      lastPrintTimeFlushing = currentTimeFlushing;
      Serial.print("Flushing... ");
      Serial.print(remainingTimeFlushing / 1000); // Print remaining time in seconds
      Serial.println(" seconds remaining");

      // Update the LVGL label
      String labelText = "Flushing... " + String(remainingTimeFlushing / 1000) + " seconds remaining";
      lv_label_set_text(ui_SerialLabel, labelText.c_str());
    }



    // Check if x seconds have passed
    if (elapsedTimeFlushing >= flushDuration) {
      digitalWrite(RELAY1, LOW);  // Turn off the output pin
      isFlushing = false;            // Reset the flushing flag
      Serial.println("Flushing ended");
       lv_label_set_text(ui_SerialLabel, "Flushing ended");
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

    Serial.print(currentWeight); // debug print weight

    // update shot trajectory
    if (shot.brewing)
    {
      shot.time_s[shot.datapoints] = seconds_f() - shot.start_timestamp_s;
      shot.weight[shot.datapoints] = currentWeight;
      shot.shotTimer = shot.time_s[shot.datapoints];
      shot.datapoints++;

      Serial.print(" ");
      Serial.print(shot.shotTimer);

      // update the UI timer label (maybe change to 100ms instead of 1)
      if (shot.shotTimer >= (previousTimerValue + 0.01))
      {
        dtostrf(shot.shotTimer, 5, 0, buffer);
        lv_label_set_text(ui_TimerLabel, buffer);
        previousTimerValue = shot.shotTimer; // Update previous timer value
      }

      // get the likely end time of the shot
      calculateEndTime(&shot);
      Serial.print(" ");
      Serial.print(shot.expected_end_s);
      
      // Update the LVGL label
      String labelText = "Expected end time @ " + String(shot.expected_end_s) + " s";
      lv_label_set_text(ui_SerialLabel, labelText.c_str());
    }
    Serial.println();
  }

  // Read button every period 
  if (millis() > (lastButtonRead_ms + BUTTON_READ_PERIOD_MS))
  {
    // save last check timing
    lastButtonRead_ms = millis();

    // push back for new entry
    for (int i = 2; i >= 0; i--)
    {
      buttonArr[i + 1] = buttonArr[i];
    }
    
    newButtonState = 0;

    for (int i = 0; i < 4; i++)
    {
      if (buttonArr[i])
      {
        newButtonState = 1;
      }
    }

    if (REEDSWITCH && !shot.brewing && seconds_f() < (shot.start_timestamp_s + shot.end_s + 0.5))
    {
      newButtonState = 0;
    }
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

  // button just pressed
  if (newButtonState && buttonPressed == false)
  {
    Serial.println("ButtonPressed");
    buttonPressed = true;
    if (!MOMENTARY)
    {
      shot.brewing = true;
      setBrewingState(shot.brewing);
    }
  }

  // button held. Take over for the rest of the shot.
  else if (!MOMENTARY && shot.brewing && !buttonLatched && (shot.shotTimer > MIN_SHOT_DURATION_S))
  {
    buttonLatched = true;
    Serial.println("Button Latched");

    // Get the scale to beep to inform user.
    scale.tare();
    // Print system status
    lv_label_set_text(ui_SerialLabel, "Brewing Now~");
  }

  // button released
  else if (!buttonLatched && !newButtonState && buttonPressed == true)
  {
    Serial.println("Button Released");
    buttonPressed = false;
    shot.brewing = !shot.brewing;
    setBrewingState(shot.brewing);
  }

  // Max duration reached
  else if (shot.brewing && shot.shotTimer > MAX_SHOT_DURATION_S)
  {
    shot.brewing = false;
    Serial.println("Max brew duration reached");
    setBrewingState(shot.brewing);
    lv_label_set_text(ui_SerialLabel, "Max brew duration reached");
  }

  // End shot
  if (shot.brewing && shot.shotTimer >= shot.expected_end_s && shot.shotTimer > MIN_SHOT_DURATION_S)
  {
    Serial.println("weight achieved");
    shot.brewing = false;
    setBrewingState(shot.brewing);
    lv_label_set_text(ui_SerialLabel, "Weight achieved");
  }

  // Detect error of shot
  if (shot.start_timestamp_s && shot.end_s && currentWeight >= (goalWeight - weightOffset) && seconds_f() > shot.start_timestamp_s + shot.end_s + DRIP_DELAY_S)
  {
    shot.start_timestamp_s = 0;
    shot.end_s = 0;

    Serial.print("I detected a final weight of ");
    Serial.print(currentWeight);
    Serial.print("g. The goal was ");
    Serial.print(goalWeight);
    Serial.print("g with a negative offset of ");
    Serial.print(weightOffset);

    if (abs(currentWeight - goalWeight + weightOffset) > MAX_OFFSET)
    {
      Serial.print("g. Error assumed. Offset unchanged. ");
      lv_label_set_text(ui_SerialLabel, "Error assumed. Offset unchanged.");
    }
    else
    {
      Serial.print("g. Next time I'll create an offset of ");
      weightOffset += currentWeight - goalWeight;
      Serial.print(weightOffset);

      // Save offset
      saveOffset(weightOffset * 10);
    }
    Serial.println();
  }
}

void setBrewingState(bool brewing)
{
  if (brewing)
  {
    Serial.println("shot started");
    shot.start_timestamp_s = seconds_f();
    shot.shotTimer = 0;
    shot.datapoints = 0;
    scale.startTimer();
    scale.tare();
    Serial.println("Weight Timer End");
    lv_label_set_text(ui_SerialLabel, "Shot started");
  }
  else
  {
    Serial.println("ShotEnded");
    lv_label_set_text(ui_SerialLabel, "Shot ended");
    shot.end_s = seconds_f() - shot.start_timestamp_s;
    scale.stopTimer();
    buttonLatched = false;
    buttonPressed = false;
    digitalWrite(RELAY1, LOW);
    Serial.println("Button Unlatched");
  }
}

void calculateEndTime(Shot *s)
{
  // Do not  predict end time if there aren't enough espresso measurements yet
  if ((s->datapoints < N) || (s->weight[s->datapoints - 1] < 10))
  {
    s->expected_end_s = MAX_SHOT_DURATION_S;
  }
  else
  {
    // Get line of best fit (y=mx+b) from the last 10 measurements
    float sumXY = 0, sumX = 0, sumY = 0, sumSquaredX = 0, m = 0, b = 0, meanX = 0, meanY = 0;

    for (int i = s->datapoints - N; i < s->datapoints; i++)
    {
      sumXY += s->time_s[i] * s->weight[i];
      sumX += s->time_s[i];
      sumY += s->weight[i];
      sumSquaredX += (s->time_s[i] * s->time_s[i]);
    }

    m = (N * sumXY - sumX * sumY) / (N * sumSquaredX - (sumX * sumX));
    meanX = sumX / N;
    meanY = sumY / N;
    b = meanY - m * meanX;

    // Calculate time at which goal weight will be reached (x = (y-b)/m)
    s->expected_end_s = (goalWeight - weightOffset - b) / m;
  }
}

float seconds_f()
{
  return millis() / 1000.0;
}