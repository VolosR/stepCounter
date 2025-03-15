#include <lvgl.h>
#include <Wire.h>
#include <Arduino.h>
#include "pin_config.h"
#include "XPowersLib.h"
#include "lv_conf.h"
#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include <ESP_IOExpander_Library.h>
#include "HWCDC.h"
#include "ui.h"
#include "ESP_I2S.h"
#include "esp_check.h"
#include "es8311.h"
#include <EEPROM.h>

#include "SensorPCF85063.hpp"
#include "SensorQMI8658.hpp"


#define EEPROM_SIZE 50
#define _EXAMPLE_CHIP_CLASS(name, ...) ESP_IOExpander_##name(__VA_ARGS__)
#define EXAMPLE_CHIP_CLASS(name, ...) _EXAMPLE_CHIP_CLASS(name, ##__VA_ARGS__)

int bri=160; // brightness
#define EXAMPLE_SAMPLE_RATE 16000
#define EXAMPLE_VOICE_VOLUME 70                // 0 - 100
#define EXAMPLE_MIC_GAIN (es8311_mic_gain_t)(3)  // 0 - 7
//#define EXAMPLE_RECV_BUF_SIZE (1000)

XPowersPMU power;
ESP_IOExpander *expander = NULL;
I2SClass i2s;


bool pmu_flag = 1;
bool adc_switch = false;

SensorQMI8658 qmi;
IMUdata acc;
IMUdata gyr;

bool intro=1;
bool sound=1;
int mode=0; // 0 is 0, 1 is runing , 2 is stoped
String modesLabel[3]={"START","STOP","RESET"};
int stepCount = 0;     // Brojač koraka
float stepDistance=0.75;
int weight=80;
float distance=0;
float lastMagnitude = 0;    
float stepThreshold = 1.8;  // Prag za detekciju koraka
String percent="";

float speed=0;
float calories=0;
float calPerStep=0.044;

unsigned long lastTouched=0;
unsigned long timeBuff=0;
bool canDetect=1;
bool soundOn=1;

HWCDC USBSerial;
#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[LCD_WIDTH * LCD_HEIGHT / 10];

static lv_obj_t * meter;
lv_meter_indicator_t * needle; 
lv_obj_t *label;  // Global label object
SensorPCF85063 rtc;
uint32_t lastMillis;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS /* CS */, LCD_SCLK /* SCK */, LCD_SDIO0 /* SDIO0 */, LCD_SDIO1 /* SDIO1 */,
  LCD_SDIO2 /* SDIO2 */, LCD_SDIO3 /* SDIO3 */);

Arduino_GFX *gfx = new Arduino_SH8601(bus, -1 /* RST */,
                                      0 /* rotation */, false /* IPS */, LCD_WIDTH, LCD_HEIGHT);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

void Arduino_IIC_Touch_Interrupt(void);
std::unique_ptr<Arduino_IIC> FT3168(new Arduino_FT3x68(IIC_Bus, FT3168_DEVICE_ADDRESS,
                                                       DRIVEBUS_DEFAULT_VALUE, TP_INT, Arduino_IIC_Touch_Interrupt));

const char *TAG = "esp32p4_i2s_es8311";

esp_err_t es8311_codec_init(void) {
  es8311_handle_t es_handle = es8311_create(0, ES8311_ADDRRES_0);
  ESP_RETURN_ON_FALSE(es_handle, ESP_FAIL, TAG, "es8311 create failed");
  const es8311_clock_config_t es_clk = {
    .mclk_inverted = false,
    .sclk_inverted = false,
    .mclk_from_mclk_pin = true,
    .mclk_frequency = EXAMPLE_SAMPLE_RATE * 256,
    .sample_frequency = EXAMPLE_SAMPLE_RATE
  };

  ESP_ERROR_CHECK(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
  ESP_RETURN_ON_ERROR(es8311_sample_frequency_config(es_handle, es_clk.mclk_frequency, es_clk.sample_frequency), TAG, "set es8311 sample frequency failed");
  ESP_RETURN_ON_ERROR(es8311_microphone_config(es_handle, false), TAG, "set es8311 microphone failed");

  ESP_RETURN_ON_ERROR(es8311_voice_volume_set(es_handle, EXAMPLE_VOICE_VOLUME, NULL), TAG, "set es8311 volume failed");
  ESP_RETURN_ON_ERROR(es8311_microphone_gain_set(es_handle, EXAMPLE_MIC_GAIN), TAG, "set es8311 microphone gain failed");
  return ESP_OK;
}

void adcOn() {
  power.enableTemperatureMeasure();
  power.enableBattDetection();
  power.enableVbusVoltageMeasure();
  power.enableBattVoltageMeasure();
  power.enableSystemVoltageMeasure();
}

void adcOff() {
  power.disableTemperatureMeasure();
  power.disableBattDetection();
  power.disableVbusVoltageMeasure();
  power.disableBattVoltageMeasure();
  power.disableSystemVoltageMeasure();
}

void Arduino_IIC_Touch_Interrupt(void) {
  FT3168->IIC_Interrupt_Flag = true;
}


/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

void example_increase_lvgl_tick(void *arg) {
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}



/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  int32_t touchX = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
  int32_t touchY = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

  if (FT3168->IIC_Interrupt_Flag == true) {
    FT3168->IIC_Interrupt_Flag = false;
    data->state = LV_INDEV_STATE_PR;

    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;

    lastTouched=millis();
    gfx->Display_Brightness(bri);

    USBSerial.print("Data x ");
    USBSerial.print(touchX);

    USBSerial.print("Data y ");
    USBSerial.println(touchY);
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void setFlag(void) {
  pmu_flag = true;
}

void lv_example_meter_1(void)
{
     meter = lv_meter_create(lv_scr_act());
    lv_obj_center(meter);
    lv_obj_set_size(meter, 120, 120);
    lv_obj_set_pos(meter, 99, 164);
    lv_obj_set_style_bg_opa(meter, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(meter, 0, LV_PART_MAIN);

    /*Add a scale first*/
    lv_meter_scale_t * scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_range(meter, scale, 0, 10, 270, 135); // Set range from 0 to 10
    lv_meter_set_scale_ticks(meter, scale, 31, 2, 10, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter, scale, 6, 4, 15, lv_color_white(), 10);

    lv_meter_indicator_t * indic;

    /*Add a blue arc to the start*/
    indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_meter_set_indicator_start_value(meter, indic, 0);
    lv_meter_set_indicator_end_value(meter, indic, 2);

    /*Make the tick lines blue at the start of the scale*/
    indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_BLUE), false, 0);
    lv_meter_set_indicator_start_value(meter, indic, 0);
    lv_meter_set_indicator_end_value(meter, indic, 2);

    /*Add a red arc to the end*/
    indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_indicator_start_value(meter, indic, 8);
    lv_meter_set_indicator_end_value(meter, indic, 10);

    /*Make the tick lines red at the end of the scale*/
    indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
    lv_meter_set_indicator_start_value(meter, indic, 8);
    lv_meter_set_indicator_end_value(meter, indic, 10);

    /*Add a needle line indicator*/
    needle = lv_meter_add_needle_line(meter, scale, 4, lv_palette_main(LV_PALETTE_GREY), -10); 
}

void setup() {
  USBSerial.begin(115200); /* prepare for possible serial debug */
  pinMode(PA, OUTPUT),
  digitalWrite(PA, HIGH);

  i2s.setPins(BCLKPIN, WSPIN, DIPIN, DOPIN, MCLKPIN);
  if (!i2s.begin(I2S_MODE_STD, EXAMPLE_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_BOTH)) {
    Serial.println("Failed to initialize I2S bus!");
    return;
  }

  Wire.begin(IIC_SDA, IIC_SCL);

  es8311_codec_init();
  if (!rtc.begin(Wire, PCF85063_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    USBSerial.println("Failed to find PCF8563 - check your wiring!");
    while (1) {
      delay(1000);
    }
  }

  rtc.setDateTime(2025, 2, 28, 0, 0, 0);
  expander = new EXAMPLE_CHIP_CLASS(TCA95xx_8bit,
                                    (i2c_port_t)0, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000,
                                    IIC_SCL, IIC_SDA);

  bool result = power.begin(Wire, AXP2101_SLAVE_ADDRESS, IIC_SDA, IIC_SCL);

  if (result == false) {
    USBSerial.println("PMU is not online...");
    while (1) delay(50);
  }

  expander->init();
  expander->begin();
  expander->pinMode(5, INPUT);
  expander->pinMode(4, INPUT);
  expander->pinMode(1, OUTPUT);
  expander->pinMode(2, OUTPUT);
  expander->digitalWrite(1, LOW);
  expander->digitalWrite(2, LOW);
  delay(20);
  expander->digitalWrite(1, HIGH);
  expander->digitalWrite(2, HIGH);

  int pmu_irq = expander->digitalRead(5);
  if (pmu_irq == 1) {
    setFlag();
  }

  power.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
  power.setChargeTargetVoltage(3);
  power.clearIrqStatus();
  power.enableIRQ(
    XPOWERS_AXP2101_PKEY_SHORT_IRQ  //POWER KEY
  );

  adcOn();

  while (FT3168->begin() == false) {
    USBSerial.println("FT3168 initialization fail");
    delay(2000);
  }
  USBSerial.println("FT3168 initialization successfully");

  //eeprom readings  ############################################################################### EEPROM
    EEPROM.begin(EEPROM_SIZE);
    //brightness
    bri=EEPROM.read(0);
    if(bri<80) bri=120;
    
    //sound
    sound=EEPROM.read(1);
    if(sound>1) sound=1;

    //sensorTreshold
    stepThreshold=EEPROM.read(2)/100.00;
    if(stepThreshold>2) stepThreshold=1.75;

    //step size
    stepDistance=EEPROM.read(3)/100.00;
    if(stepDistance<0.40 || stepDistance>2)
    stepDistance=0.8;

    //weight
    weight=EEPROM.read(4);
    if(weight>200)
    weight=70;

  gfx->begin();
  gfx->Display_Brightness(bri);
  lv_init();

  FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                 FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR);

  lv_disp_draw_buf_init(&draw_buf, buf, NULL, LCD_WIDTH * LCD_HEIGHT / 10);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = LCD_WIDTH;
  disp_drv.ver_res = LCD_HEIGHT;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick,
    .name = "lvgl_tick"
  };

  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    Serial.println("Failed to find QMI8658 - check your wiring!");
    while (1) { delay(1000); }
  }

  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz, SensorQMI8658::LPF_MODE_0);
  qmi.enableAccelerometer();

  ui_init();
  lv_example_meter_1();

  lv_slider_set_value(ui_brightSLI, bri, LV_ANIM_OFF);
  if(sound)
  lv_obj_add_flag(ui_sndPNL, LV_OBJ_FLAG_HIDDEN);

  lv_label_set_text(ui_tresLBL,String(stepThreshold).c_str());
  lv_label_set_text(ui_tresLBL2,String(stepThreshold).c_str());
  lv_label_set_text(ui_StepdisLBL,String(stepDistance).c_str());
  lv_label_set_text(ui_StepdisLBL1,String(stepDistance).c_str());
  lv_label_set_text(ui_weightLBL,String(weight).c_str());
  lv_label_set_text(ui_weightLBL2,String(weight).c_str()); 

  
  lv_scr_load(ui_Screen2);

}

void briChanged(lv_event_t * e)
{
  
  bri=lv_slider_get_value(ui_brightSLI);
  EEPROM.write(0,bri); EEPROM.commit();
  gfx->Display_Brightness(bri);
}

void setTreshold(lv_event_t * e)
{
     lv_obj_t * btn = lv_event_get_target(e);
     if(btn==ui_Button1)
     {stepThreshold=stepThreshold+0.05;
     lv_label_set_text(ui_tresLBL,String(stepThreshold).c_str()); 
     lv_label_set_text(ui_tresLBL2,String(stepThreshold).c_str()); 
     EEPROM.write(2,stepThreshold*100); EEPROM.commit();
     if(sound)
     beep(2400,50);
     }
      if(btn==ui_Button2)
     {stepThreshold=stepThreshold-0.05;
     lv_label_set_text(ui_tresLBL,String(stepThreshold).c_str()); 
     lv_label_set_text(ui_tresLBL2,String(stepThreshold).c_str()); 
     EEPROM.write(2,stepThreshold*100); EEPROM.commit();
      if(sound)
      beep(2400,50);
     }
     if(btn==ui_Button3)
     {
      mode++; if(mode>2) mode=0;
      lv_label_set_text(ui_Label5,String(modesLabel[mode]).c_str());
      if(mode==0)
      {
       if(sound)
       beep(2400,60);
       lv_label_set_text(ui_Label1, "00:00");
       lv_label_set_text(ui_secLBL, "00");
       stepCount=0;
      
      lv_label_set_text(ui_stepLBL,"0"); 
      lv_label_set_text(ui_disLBL, "0.00");
      lv_label_set_text(ui_caloriesLBL,"0.00");
      lv_label_set_text(ui_speedLBL,"0.00");
      }
       if(mode==1)
      {
      if(sound)
      beep(2400,60);
      lv_obj_clear_flag(ui_Spinner2, LV_OBJ_FLAG_HIDDEN);
      rtc.setDateTime(2025, 2, 28, 0, 0, 0);
      }
       if(mode==2)
      {
       if(sound) 
       beep(2400,60);
       lv_obj_add_flag(ui_Spinner2, LV_OBJ_FLAG_HIDDEN);
      }
     }
}

/*
void changeVolume(lv_event_t * e)
{
  int vol=lv_slider_get_value(ui_Slider1);
 // es8311_voice_volume_set(es_handle, vol, NULL)
  lv_label_set_text(ui_volumeLBL,String(vol).c_str());
}
*/
void soundOnOFF(lv_event_t * e)
{
   lv_obj_t * btn = lv_event_get_target(e);
   if(btn==ui_soundOnBTN)
     {
      sound=1;
      lv_label_set_text(ui_soundLBL,"SOUND IS ON");
      lv_obj_add_flag(ui_sndPNL, LV_OBJ_FLAG_HIDDEN);
     }
     if(btn==ui_soundOffBTN)
     {
      sound=0;
      lv_label_set_text(ui_soundLBL,"SOUND IS OFF");
      lv_obj_clear_flag(ui_sndPNL, LV_OBJ_FLAG_HIDDEN);
     }
     EEPROM.write(1,sound); EEPROM.commit();
     if(sound)
     beep(2400,50);
}


void setSDist(lv_event_t * e)
{
     lv_obj_t * btn = lv_event_get_target(e);
     if(btn==ui_Button6)
     {stepDistance=stepDistance+0.05;
     lv_label_set_text(ui_StepdisLBL,String(stepDistance).c_str()); 
     lv_label_set_text(ui_StepdisLBL1,String(stepDistance).c_str()); 
     EEPROM.write(3,stepDistance*100); EEPROM.commit();
     }
      if(btn==ui_Button7)
     {stepDistance=stepDistance-0.05;
     lv_label_set_text(ui_StepdisLBL,String(stepDistance).c_str()); 
     lv_label_set_text(ui_StepdisLBL1,String(stepDistance).c_str()); 
     EEPROM.write(3,stepDistance*100); EEPROM.commit();
     }   
     if(sound)
     beep(2400,50);
}

void setWeight(lv_event_t * e)
{
     lv_obj_t * btn = lv_event_get_target(e);
     if(btn==ui_weightUP)
     {weight++;
     lv_label_set_text(ui_weightLBL,String(weight).c_str()); 
     lv_label_set_text(ui_weightLBL2,String(weight).c_str()); 
     EEPROM.write(4,weight); EEPROM.commit();
     }

     if(btn==ui_weightDOWN)
     {weight--;
     lv_label_set_text(ui_weightLBL,String(weight).c_str()); 
     lv_label_set_text(ui_weightLBL2,String(weight).c_str()); 
     EEPROM.write(4,weight); EEPROM.commit();
     }
    if(sound)
    beep(2400,50);
}

void beep(int frequency, int duration) {
    const int SAMPLE_RATE = 16000;
    const int AMPLITUDE = 8000;
    const float TWO_PI2 = 6.283185;
    long samples = (SAMPLE_RATE * duration) / 1000;
    unsigned char buffer[samples];

    for (int i = 0; i < samples; i++) {
        buffer[i] = AMPLITUDE * sin((TWO_PI2 * frequency * i) / SAMPLE_RATE);
    }

    size_t bytes_written;
    i2s.write((uint8_t *)buffer, samples);
    delay(200);
}

float calculateSpeed(int hours, int minutes, int seconds, float distance_meters) {
    // Pretvaranje ukupnog vremena u sate
    float time_hours = hours + (minutes / 60.0) + (seconds / 3600.0);
    
    // Pretvaranje udaljenosti u kilometre
    float distance_km = distance_meters / 1000.0;
    
    // Izračun brzine (km/h)
    if (time_hours > 0) {
        return distance_km / time_hours;
    } else {
        return 0; // Ako je vrijeme 0, izbjegavamo dijeljenje s nulom
    }
}

float calculate_calories(int broj_koraka, float tezina_kg) {
    float faktor_potrosnje = 0.0005 * tezina_kg; // Približna vrijednost
    return broj_koraka * faktor_potrosnje;
}



void loop() {

  if(intro==1)
  if(millis()>3000)
  {
    intro=0;
    lv_scr_load(ui_Screen1);
  }

  lv_timer_handler(); /* let the GUI do its work */
  delay(5);

  if(power.isCharging())
  {
  lv_obj_clear_flag(ui_crgPNL, LV_OBJ_FLAG_HIDDEN);
  }
  else
  {
  lv_obj_add_flag(ui_crgPNL, LV_OBJ_FLAG_HIDDEN);
  }

  lv_bar_set_value(ui_sleepBar, lastTouched+8000-millis(), LV_ANIM_OFF);
  if(millis()>lastTouched+8000)
  {
     gfx->Display_Brightness(0);
  }
  
  if(canDetect==0 && millis()>timeBuff+100)
  {
    canDetect=1;
  }

    uint32_t status = power.getIrqStatus();
    if (power.isPekeyShortPressIrq()) {
        adcOff();
        power.shutdown();
    }
    power.clearIrqStatus();

if(mode==1){
if (qmi.getDataReady()) {
    if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
        USBSerial.print("{ACCEL: ");
        USBSerial.print(acc.x);
     
 
        float magnitude = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);

        // Detekcija koraka – poređenje sa pragom
        if (lastMagnitude < stepThreshold && magnitude >= stepThreshold && canDetect==1) {
            stepCount++;
            if(sound)
            if(stepCount<30)
            beep( 2500,  120);
            distance=stepCount*stepDistance;
            calories=calculate_calories(stepCount, weight);
            canDetect=0;
            timeBuff=millis();

            RTC_DateTime datetime = rtc.getDateTime();
            speed=calculateSpeed(datetime.hour,datetime.minute,datetime.second,distance);
           
            lv_label_set_text(ui_stepLBL,  String(stepCount).c_str());
            lv_label_set_text(ui_disLBL, String(distance).c_str());
            lv_label_set_text(ui_caloriesLBL,String(calories).c_str());
            lv_label_set_text(ui_speedLBL,String(speed).c_str());
            lv_meter_set_indicator_value(meter, needle, speed);
        }
        lastMagnitude = magnitude;  
    }
}


  if (millis() - lastMillis > 1000) {
    lastMillis = millis();
    RTC_DateTime datetime = rtc.getDateTime();

    char buf[16];
    snprintf(buf, sizeof(buf), "%02d:%02d",datetime.hour, datetime.minute);
    char buf2[4];
    snprintf(buf2, sizeof(buf2), "%02d", datetime.second);

    // Update label with current time
    
    lv_label_set_text(ui_Label1, buf);
    lv_label_set_text(ui_secLBL, buf2);
   
  }}

  percent=String(power.getBatteryPercent())+"%";
  lv_label_set_text(ui_batLBL,percent.c_str());  
  
}
