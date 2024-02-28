#include <EEPROM.h> 

#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;

TaskHandle_t Task1 = NULL;
TaskHandle_t Task2 = NULL;

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
byte tlogo0[] = { B00000, B00001, B00010, B00100, B00100, B00100, B00100, B00111 };
byte tlogo1[] = { B00000, B10000, B01011, B00100, B00111, B00100, B00111, B11100 };
byte tlogo2[] = { B00111, B00111, B00111, B01111, B11111, B11111, B01111, B00011 };
byte tlogo3[] = { B11111, B11100, B11100, B11110, B11111, B11111, B11110, B11000 };
byte degree[] = { B11000, B11000, B00000, B00111, B01000, B01000, B01000, B00111 };
byte arrow0[] = { B00000, B10000, B11000, B11100, B11100, B11000, B10000, B00000 };
byte arrow1[] = { B00000, B00000, B00000, B01111, B01111, B00000, B00000, B00000 };

//Encoder 200p/r
#include <driver/pcnt.h>
hw_timer_t* timer = NULL;
double _Ticks = 0.00;
int stby_en = 0;
#define _pinA 13
#define _pinB 12

//ky
#define sw 14
#define A 27
#define B 26
int A_set = 0;
int count = 0;
int count_o = 1;
int menu = 0, list = 0;
char keybord[76] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789+-*/.!@#$%^& ";
template< typename T, size_t N > size_t ArraySize(T (&)[N]) {
  return N;
}
String menumessege[] = { "..              ",
                         "Presets        >",
                         "Temperature    >",
                         "Motor          >",
                         "History        >",
                         "Setup          >",
                         "Save config    >" };

String ready_menu[] = { "..              ",
                        "Start           ",
                        "Cancel          ",
                        "Temperature    >",
                        "Motor          >",
                        "History        >" };

String setup_menu[] = { "..         (Setup)",
                        "Restore        >  ",
                        "PID tuning     >  ",
                        "Encoder tuning >  ",
                        "Proximity      >  ",
                        "Buzzer         >  ",
                        "Fan            >  " };

String presets_menu[] = { "..       (Presets)",
                          "                  ",
                          "                  ",
                          "                  ",
                          "                  " };

String history_menu[] = { "..       (History)",
                          "                  ",
                          "                  ",
                          "                  ",
                          "                  " };

//temp
double Beta = 3950.0;
const int temp_pin = 16;
const int pid_Ch = 0;

int set_temp = 0;
int Max_temp = 260;
int Chamber_temp = 20;
double Kp = 37.0;
double Ki = 1.5;
double Kd = 80.0;

// double Kp = 84.0;
// double Ki = 5.0;
// double Kd = 220.0;
double temp, PID_Val, error, pre_error, integral, derivative;
unsigned long lastTime = 0;
unsigned long sampleTime = 1000;

//Motor
#define STBY 17     //standby
#define PWM_pin 19  //Speed control
#define AIN1 18     //Direction
#define AIN2 5      //Direction
const int PWM_Motor = 1;
int set_speed = 0;
int Max_speed = 100;

//Buzzer
#define buzzer_pin 32
int buzzer_ch = 2;
int buzzer_on_off = 0;

int proximity = 0;
int proximity_check = 0;

unsigned long last_alarmBuzzer = 0;
bool buzzerActive = false;

int ff = 0;

bool undetected = false;

//millis
int t1 = 1000;
unsigned long last_t1 = 0;
int t2 = 600;
unsigned long last_t2 = 0;

int t4 = 1000;
unsigned long last_t4 = 0;
int testpid = 0;

int prox = 500;
unsigned long last_prox = 0;

int t_name = 250;  //เปลี่ยนชื่อโปรไฟล์
unsigned long last_t_name = 0;

//พักหน้าจอ
int t3 = 1000;
unsigned long last_t3 = 0;
int count_time = 0;
int time_t3 = 20;  // 30วินาที

int j = 0, k = 0, k_memu = 0, setup_memu = 0, presets_menu_en = 0, history_menu_en = 0;
int ready = 0, start_stop = 0;

//function
String floattostring(float num, int mode = 0);

//interrupt PCNT Encoder
void IRAM_ATTR Update_Ticks() {
  int16_t c;
  pcnt_get_counter_value(PCNT_UNIT_0, &c);  //Get pulse counter value.
  if ((_Ticks + c) != _Ticks) {
    pcnt_counter_clear(PCNT_UNIT_0);                 //Clear and reset PCNT counter value to zero.
    double radius_en = 100.531;                      //ความยาวรอบวงของแกนที่หมุน = 2 × π × รัศมีของวงกลม(16mm) 2 * 3.1416 * 16
    double rpm_en = c / 200.0;                       //จำนวนรอบ = จำนวน pulses ที่ได้รับ / 200 pulses/รอบ **จับทั้ง AB 400 pulses/รอบ
    double lenth_en = (rpm_en * radius_en) * 0.001;  //ความยาวที่วัดได้ = จำนวนรอบ × ความยาวรอบวงของแกนที่หมุน **แปลงหน่วยเป็นเมตร
    if (stby_en == 1) _Ticks = _Ticks + lenth_en;    //อัพเดตความยาว
  }
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  delay(200);

  //temp
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1)
      ;
  }
  ledcSetup(pid_Ch, 5000, 8);
  ledcAttachPin(temp_pin, pid_Ch);

  //ky
  pinMode(sw, INPUT_PULLUP);
  pinMode(A, INPUT);
  pinMode(B, INPUT);

  //Motor
  ledcSetup(PWM_Motor, 5000, 8);
  ledcAttachPin(PWM_pin, PWM_Motor);
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  //buzzer
  ledcSetup(buzzer_ch, 2000, 8);
  ledcAttachPin(buzzer_pin, buzzer_ch);

  lcd.begin();
  lcd.clear();
  lcd.createChar(0, tlogo0);
  lcd.createChar(1, tlogo1);
  lcd.createChar(2, tlogo2);
  lcd.createChar(3, tlogo3);
  lcd.createChar(4, degree);
  lcd.createChar(5, arrow0);
  lcd.createChar(6, arrow1);

  //proximity
  pinMode(4, INPUT_PULLDOWN);

  //history
  int address = 0;
  float address1 = EEPROM.readFloat(address);
  address += sizeof(float);
  float address2 = EEPROM.readFloat(address);
  address += sizeof(float);
  float address3 = EEPROM.readFloat(address);
  address += sizeof(float);
  float address4 = EEPROM.readFloat(address);
  address += sizeof(float);
  history_menu[1] = floattostring(address1, 1);
  history_menu[2] = floattostring(address2);
  history_menu[3] = floattostring(address3);
  history_menu[4] = floattostring(address4);

  String name1 = EEPROM.readString(16);
  String name2 = EEPROM.readString(35);
  String name3 = EEPROM.readString(54);
  String name4 = EEPROM.readString(73);
  presets_menu[1] = name_eeprom(name1);
  presets_menu[2] = name_eeprom(name2);
  presets_menu[3] = name_eeprom(name3);
  presets_menu[4] = name_eeprom(name4);

  xTaskCreatePinnedToCore(Task1code, "Task1", 2000, NULL, 1, &Task1, 0);
  delay(500);
  xTaskCreatePinnedToCore(Task2code, "Task2", 3000, NULL, 2, &Task2, 1);
  delay(500);
}

void Task1code(void* pvParameters) {
  /* Use PCNT Encoder */
  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);
  pcnt_config_t encoder = {
    .pulse_gpio_num = _pinA,          //Pulse input GPIO number
    .ctrl_gpio_num = _pinB,           //Control signal input GPIO number
    .lctrl_mode = PCNT_MODE_REVERSE,  //PCNT low control mode, invert counter mode(increase -> decrease, decrease -> increase)
    .hctrl_mode = PCNT_MODE_KEEP,     //PCNT high control mode, won’t change counter mode
    .pos_mode = PCNT_COUNT_DIS,       //PCNT positive edge count mode, Decrease counter value
    .neg_mode = PCNT_COUNT_INC,       //PCNT negative edge count mode,Increase counter value
    .counter_h_lim = 32766,           //Maximum counter value  16-bit
    .counter_l_lim = -32766,          //Minimum counter value  16-bit
    .unit = PCNT_UNIT_0,              //PCNT unit 0
    .channel = PCNT_CHANNEL_0,        //PCNT channel 0
  };
  pcnt_unit_config(&encoder);                      //Configure Pulse Counter unit.
  pcnt_set_filter_value(PCNT_UNIT_0, 250);         //Set PCNT filter value Default 250
  pcnt_filter_enable(PCNT_UNIT_0);                 //Enable PCNT input filter.
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);  //Enable PCNT watch point event: Maximum counter value
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);  //Enable PCNT watch point event: Minimum counter value
  pcnt_counter_pause(PCNT_UNIT_0);                 //Pause PCNT counter of PCNT unit.
  pcnt_counter_clear(PCNT_UNIT_0);                 //Clear and reset PCNT counter value to zero.
  pcnt_intr_enable(PCNT_UNIT_0);                   //Enable PCNT interrupt for PCNT unit.
  pcnt_counter_resume(PCNT_UNIT_0);                //Resume counting for PCNT counter.

  /* Use 0st timer of 4 */
  timer = timerBegin(0, 80, true);                   // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer, &Update_Ticks, true);  // edge (not level) triggered
  timerAlarmWrite(timer, 1000000, true);             // 1000000 * 1 us = 1 s, autoreload true
  timerAlarmEnable(timer);                           // enable

  for (;;) {
    Pid_Temp();
    Proximity();
    Buzzer_proximity();

    //Serial.println(temp);

    // Serial.println(temp);
    // Serial.print(Beta);
    // Serial.print(count);
    // Serial.print("\t    j: ");
    // Serial.print(j);
    // Serial.print("\t    k_memu: ");
    // Serial.print(k_memu);

    // Serial.print("\t    prox: ");
    //Serial.println(proximity);
  }
}

void Task2code(void* pvParameters) {
  for (;;) {
    while (menu == 0) {
      if (j == 0) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.write(0);
        lcd.write(1);

        lcd.setCursor(0, 1);
        lcd.write(2);
        lcd.write(3);

        lcd.setCursor(10, 0);
        lcd.print("L:");
        lcd.setCursor(10, 1);
        lcd.print("M:");
        lcd.print(set_speed);
        lcd.print("% ");

        j = 1;
      }

      if (millis() - last_t1 > t1) {
        last_t1 = millis();
        lcd.setCursor(3, 0);
        lcd.print(temp, 1);
        lcd.write(4);
        lcd.print(" ");

        Status_memu();  //สถานะเครื่อง

        lcd.setCursor(3, 1);
        lcd.print(set_temp);
        lcd.write(4);
        lcd.print("  ");

        lcd.setCursor(12, 0);
        lcd.print(_Ticks);
        lcd.print("m ");
      }
      while (millis() - last_t2 > t2) {
        last_t2 = millis();
        if (k == 0) {
          lcd.setCursor(19, 3);
          lcd.print(" ");
          k = 1;
          break;
        } else {
          lcd.setCursor(19, 3);
          lcd.print("*");
          k = 0;
          break;
        }
      }

      if (digitalRead(sw) == 0 && menu == 0 && temp >= Chamber_temp) {
        while (digitalRead(sw) == 0 && menu == 0 && temp >= Chamber_temp)
          ;
        count = 0;
        A_set = digitalRead(A);
        count_o = 1;
        menu = -1;
        k_memu = 0;
        lcd.clear();
      }
    }

    if (ready == 0) {
      while (menu == -1) {
        ky();
        if (count < 0) count = 0;
        if (count > (ArraySize(menumessege) - 1)) count = (ArraySize(menumessege) - 1);
        if (count_o != count) {
          if (count < count_o) k_memu--;
          if (count > count_o) k_memu++;
          if (k_memu < 0) k_memu = 0;
          if (k_memu > 3) k_memu = 3;

          if (k_memu > 0) {
            lcd.setCursor(0, k_memu - 1);
            lcd.print("  ");
          }
          if (k_memu < 3) {
            lcd.setCursor(0, k_memu + 1);
            lcd.print("  ");
          }
          if (k_memu == 0) {
            lcd.setCursor(0, k_memu);
            lcd.write(6);
            lcd.write(5);
            lcd.setCursor(2, 0);
            lcd.print(menumessege[count]);
            lcd.setCursor(2, 1);
            lcd.print(menumessege[count + 1]);
            lcd.setCursor(2, 2);
            lcd.print(menumessege[count + 2]);
            lcd.setCursor(2, 3);
            lcd.print(menumessege[count + 3]);
          } else if (k_memu == 2) {
            lcd.setCursor(0, k_memu);
            lcd.write(6);
            lcd.write(5);
            lcd.setCursor(2, 0);
            lcd.print(menumessege[count - 2]);
            lcd.setCursor(2, 1);
            lcd.print(menumessege[count - 1]);
            lcd.setCursor(2, 2);
            lcd.print(menumessege[count]);
            lcd.setCursor(2, 3);
            lcd.print(menumessege[count + 1]);
          } else if (k_memu == 1) {
            lcd.setCursor(0, k_memu);
            lcd.write(6);
            lcd.write(5);
            lcd.setCursor(2, 0);
            lcd.print(menumessege[count - 1]);
            lcd.setCursor(2, 1);
            lcd.print(menumessege[count]);
            lcd.setCursor(2, 2);
            lcd.print(menumessege[count + 1]);
            lcd.setCursor(2, 3);
            lcd.print(menumessege[count + 2]);
          } else if (k_memu == 3) {
            lcd.setCursor(0, k_memu);
            lcd.write(6);
            lcd.write(5);
            lcd.setCursor(2, 0);
            lcd.print(menumessege[count - 3]);
            lcd.setCursor(2, 1);
            lcd.print(menumessege[count - 2]);
            lcd.setCursor(2, 2);
            lcd.print(menumessege[count - 1]);
            lcd.setCursor(2, 3);
            lcd.print(menumessege[count]);
          }
        }

        sc_time();
        count_o = count;

        if (digitalRead(sw) == 0 && menu == -1) {
          while (digitalRead(sw) == 0 && menu == -1)
            ;
          j = 0;
          menu = count;
          count = 0;
          A_set = digitalRead(A);
          count_o = 50;
          lcd.clear();
          break;
        }
      }

      while (menu == 1) {  //Presets
        presets_function();
      }
      while (menu == 2) {  //Temperature
        temperature_function();
      }
      while (menu == 3) {  //Motor
        motor_function();
      }
      while (menu == 4) {  //History
        history_function();
      }
      while (menu == 5) {  //Setup
        setup_function();
      }
      while (menu == 6) {  //Save config
        save_config_function();
      }

    } else {  //เมื่อเครื่องทำงาน
      while (menu == -1) {
        ky();
        if (count < 0) count = 0;
        if (count > (ArraySize(ready_menu) - 1)) count = (ArraySize(ready_menu) - 1);
        if (count_o != count) {
          if (count < count_o) k_memu--;
          if (count > count_o) k_memu++;
          if (k_memu < 0) k_memu = 0;
          if (k_memu > 3) k_memu = 3;

          if (k_memu > 0) {
            lcd.setCursor(0, k_memu - 1);
            lcd.print("  ");
          }
          if (k_memu < 3) {
            lcd.setCursor(0, k_memu + 1);
            lcd.print("  ");
          }
          if (k_memu == 0) {
            lcd.setCursor(0, k_memu);
            lcd.write(6);
            lcd.write(5);
            lcd.setCursor(2, 0);
            lcd.print(ready_menu[count]);
            lcd.setCursor(2, 1);
            lcd.print(ready_menu[count + 1]);
            lcd.setCursor(2, 2);
            lcd.print(ready_menu[count + 2]);
            lcd.setCursor(2, 3);
            lcd.print(ready_menu[count + 3]);
          } else if (k_memu == 2) {
            lcd.setCursor(0, k_memu);
            lcd.write(6);
            lcd.write(5);
            lcd.setCursor(2, 0);
            lcd.print(ready_menu[count - 2]);
            lcd.setCursor(2, 1);
            lcd.print(ready_menu[count - 1]);
            lcd.setCursor(2, 2);
            lcd.print(ready_menu[count]);
            lcd.setCursor(2, 3);
            lcd.print(ready_menu[count + 1]);
          } else if (k_memu == 1) {
            lcd.setCursor(0, k_memu);
            lcd.write(6);
            lcd.write(5);
            lcd.setCursor(2, 0);
            lcd.print(ready_menu[count - 1]);
            lcd.setCursor(2, 1);
            lcd.print(ready_menu[count]);
            lcd.setCursor(2, 2);
            lcd.print(ready_menu[count + 1]);
            lcd.setCursor(2, 3);
            lcd.print(ready_menu[count + 2]);
          } else if (k_memu == 3) {
            lcd.setCursor(0, k_memu);
            lcd.write(6);
            lcd.write(5);
            lcd.setCursor(2, 0);
            lcd.print(ready_menu[count - 3]);
            lcd.setCursor(2, 1);
            lcd.print(ready_menu[count - 2]);
            lcd.setCursor(2, 2);
            lcd.print(ready_menu[count - 1]);
            lcd.setCursor(2, 3);
            lcd.print(ready_menu[count]);
          }
        }

        count_o = count;

        if (digitalRead(sw) == 0 && menu == -1 && count > 2) {
          while (digitalRead(sw) == 0 && menu == -1 && count > 2)
            ;
          menu = count;
          count = 0;
          A_set = digitalRead(A);
          count_o = 50;
          lcd.clear();
          break;
        }
        if (digitalRead(sw) == 0 && menu == -1 && count == 0) {
          while (digitalRead(sw) == 0 && menu == -1 && count == 0)
            ;
          j = 0;
          menu = count;
          count = 0;
          A_set = digitalRead(A);
          count_o = -1;
          lcd.clear();
          break;
        }
        if (digitalRead(sw) == 0 && menu == -1 && count == 1) {
          while (digitalRead(sw) == 0 && menu == -1 && count == 1)
            ;
          if (start_stop == 0 && temp > set_temp - 5 && temp < set_temp + 5 && proximity == 1) {
            stby_en = 1;
            proximity_check = 1;
            ready_menu[1] = "Stop            ";
            count_o = -1;
            k_memu--;
            Motor(set_speed);

            ff = 0;  //Buzzer_proximity

            start_stop = 1;
          } else if (ready_menu[1] == "Stop            ") {
            proximity_check = 0;
            ready_menu[1] = "Start           ";
            stby_en = 0;
            count_o = -1;
            k_memu--;
            Motor(0);


            start_stop = 0;
          } else if (ready_menu[1] == "Stop alarm      ") {
            proximity_check = 0;
            ready_menu[1] = "Start           ";
            count_o = -1;
            k_memu--;
            ledcWrite(buzzer_ch, 0);
            undetected = false;
          }
        }
        if (digitalRead(sw) == 0 && menu == -1 && count == 2) {
          while (digitalRead(sw) == 0 && menu == -1 && count == 2)
            ;
          proximity_check = 0;

          set_speed = 0;
          Motor(0);
          ledcWrite(buzzer_ch, 0);
          undetected = false;
          set_temp = 0;
          ready = 0;
          j = 0;
          menu = 0;
          count = 0;
          A_set = digitalRead(A);
          count_o = -1;
          lcd.clear();
          break;
        }
      }

      while (menu == 3) {  //Temperature
        temperature_function();
      }
      while (menu == 4) {  //Motor
        motor_function();
      }
      while (menu == 5) {  //History
        history_function();
      }
    }
  }
}

void loop() {}

void presets_function() {
  ky();
  if (count < 0) count = 0;
  if (count > (ArraySize(presets_menu) - 1)) count = (ArraySize(presets_menu) - 1);
  if (count_o != count) {
    if (count < count_o) presets_menu_en--;
    if (count > count_o) presets_menu_en++;
    if (presets_menu_en < 0) presets_menu_en = 0;
    if (presets_menu_en > 3) presets_menu_en = 3;

    if (presets_menu_en > 0) {
      lcd.setCursor(0, presets_menu_en - 1);
      lcd.print("  ");
    }
    if (presets_menu_en < 3) {
      lcd.setCursor(0, presets_menu_en + 1);
      lcd.print("  ");
    }
    if (presets_menu_en == 0) {
      lcd.setCursor(0, presets_menu_en);
      lcd.write(6);
      lcd.write(5);
      lcd.setCursor(2, 0);
      lcd.print(presets_menu[count]);
      lcd.setCursor(2, 1);
      lcd.print(presets_menu[count + 1]);
      lcd.setCursor(2, 2);
      lcd.print(presets_menu[count + 2]);
      lcd.setCursor(2, 3);
      lcd.print(presets_menu[count + 3]);
    } else if (presets_menu_en == 2) {
      lcd.setCursor(0, presets_menu_en);
      lcd.write(6);
      lcd.write(5);
      lcd.setCursor(2, 0);
      lcd.print(presets_menu[count - 2]);
      lcd.setCursor(2, 1);
      lcd.print(presets_menu[count - 1]);
      lcd.setCursor(2, 2);
      lcd.print(presets_menu[count]);
      lcd.setCursor(2, 3);
      lcd.print(presets_menu[count + 1]);
    } else if (presets_menu_en == 1) {
      lcd.setCursor(0, presets_menu_en);
      lcd.write(6);
      lcd.write(5);
      lcd.setCursor(2, 0);
      lcd.print(presets_menu[count - 1]);
      lcd.setCursor(2, 1);
      lcd.print(presets_menu[count]);
      lcd.setCursor(2, 2);
      lcd.print(presets_menu[count + 1]);
      lcd.setCursor(2, 3);
      lcd.print(presets_menu[count + 2]);
    } else if (presets_menu_en == 3) {
      lcd.setCursor(0, presets_menu_en);
      lcd.write(6);
      lcd.write(5);
      lcd.setCursor(2, 0);
      lcd.print(presets_menu[count - 3]);
      lcd.setCursor(2, 1);
      lcd.print(presets_menu[count - 2]);
      lcd.setCursor(2, 2);
      lcd.print(presets_menu[count - 1]);
      lcd.setCursor(2, 3);
      lcd.print(presets_menu[count]);
    }
  }
  count_o = count;
  if (digitalRead(sw) == 0 && count == 0) {
    while (digitalRead(sw) == 0 && count == 0)
      ;
    lcd.clear();
    menu = -1;
    count = 1;
    A_set = digitalRead(A);
    count_o = -1;
    k_memu--;
  }
  if (digitalRead(sw) == 0 && count == 1) {  //Presets 1
    while (digitalRead(sw) == 0 && count == 1)
      ;
    lcd.clear();
    count_o = -1;
    count = 1;
    A_set = digitalRead(A);

    int temp_p = EEPROM.readInt(27);  //รอ EEPROM
    int motor_p = EEPROM.readInt(31);
    String name_p = EEPROM.readString(16);
    for (int i = name_p.length(); i < 10; i++)
      name_p += " ";
    char num[11];
    name_p.toCharArray(num, 11);

    while (true) {
      ky();
      if (count < 1) count = 1;
      if (count > 3) count = 3;
      if (count_o != count) {
        if (count > 0) {
          lcd.setCursor(0, count - 1);
          lcd.print("  ");
        }
        if (count < 3) {
          lcd.setCursor(0, count + 1);
          lcd.print("  ");
        }
        lcd.setCursor(0, count);
        lcd.write(6);
        lcd.write(5);
        lcd.setCursor(2, 0);
        lcd.print("(T:");
        lcd.print(temp_p);
        lcd.write(4);
        lcd.print("  ");
        lcd.print("M:");
        lcd.print(motor_p);
        lcd.print("%) ");
        lcd.setCursor(2, 1);
        lcd.print(".. ");
        lcd.setCursor(2, 2);
        lcd.print("Warm up");
        lcd.setCursor(2, 3);
        lcd.print("Edit          >");
      }
      count_o = count;
      if (digitalRead(sw) == 0 && count == 1) {
        while (digitalRead(sw) == 0 && count == 1)
          ;
        lcd.clear();
        count = 1;
        A_set = digitalRead(A);
        count_o = -1;
        presets_menu_en--;
        break;
      }
      if (digitalRead(sw) == 0 && count == 2) {
        while (digitalRead(sw) == 0 && count == 2)
          ;
        lcd.clear();
        testpid = 1;
        menu = 0;
        count = 0;
        A_set = digitalRead(A);
        count_o = -1;
        //temp motor
        set_temp = temp_p;
        set_speed = motor_p;
        ready = 1;
        break;
      }
      if (digitalRead(sw) == 0 && count == 3) {
        while (digitalRead(sw) == 0 && count == 3)
          ;
        lcd.clear();
        count = 0;
        A_set = digitalRead(A);
        count_o = -1;
        k_memu = 0;
        while (true) {
          ky();
          if (count < 0) count = 0;
          if (count > 3) count = 3;
          if (count_o != count) {
            if (count > 0) {
              lcd.setCursor(0, count - 1);
              lcd.print("  ");
            }
            if (count < 3) {
              lcd.setCursor(0, count + 1);
              lcd.print("  ");
            }
            lcd.setCursor(0, count);
            lcd.write(6);
            lcd.write(5);
            lcd.setCursor(2, 0);
            lcd.print("..");
            lcd.setCursor(2, 1);
            lcd.print("Name: ");
            lcd.print(num);
            lcd.setCursor(2, 2);
            lcd.print("Temp: ");
            lcd.print(temp_p);
            lcd.write(4);
            lcd.setCursor(2, 3);
            lcd.print("Motor: ");
            lcd.print(motor_p);
            lcd.print("%");
          }
          count_o = count;
          if (digitalRead(sw) == 0 && count == 0) {
            while (digitalRead(sw) == 0 && count == 0)
              ;
            lcd.clear();
            menu = -1;
            count = 3;
            count_o = -1;
            A_set = digitalRead(A);
            break;
          }
          if (digitalRead(sw) == 0 && count == 1) {
            while (digitalRead(sw) == 0 && count == 1)
              ;
            A_set = digitalRead(A);
            count_o = 1;
            int name_k = 0;
            count = 0;

            unsigned long buttonPressTime = 0;
            while (true) {
              ky();
              if (count < 0) count = 0;
              if (count > 9) count = 9;
              if (count_o != count) {
                lcd.setCursor(7, 1);
                lcd.print(" ");
                lcd.print(num);
                lcd.print(" ");
              }
              count_o = count;

              while (millis() - last_t_name > t_name) {
                last_t_name = millis();
                if (name_k == 0) {
                  lcd.setCursor(count + 8, 1);
                  lcd.print(" ");
                  name_k = 1;
                  break;
                } else {
                  lcd.setCursor(count + 8, 1);
                  lcd.print(num[count]);
                  name_k = 0;
                  break;
                }
              }
              if (digitalRead(sw) == 0) {
                buttonPressTime = millis();
                int lcd_count = count + 7;
                int count_1 = count;
                while (digitalRead(sw) == 0) {
                  if (millis() - buttonPressTime >= 1500) {
                    for (int i = 0; i < 75; i++)
                      if (keybord[i] == num[count]) {
                        count = i;
                        break;
                      }
                    lcd.setCursor(lcd_count, 1);
                    lcd.print("(");
                    lcd.print(num[count_1]);
                    lcd.print(")");
                    while (digitalRead(sw) == 0)
                      ;
                    while (true) {
                      ky();
                      if (count < 0) count = 74;
                      if (count > 74) count = 0;

                      while (millis() - last_t_name > t_name) {
                        last_t_name = millis();
                        if (name_k == 0) {
                          lcd.setCursor(lcd_count, 1);
                          lcd.print("( )");
                          name_k = 1;
                          break;
                        } else {
                          lcd.setCursor(lcd_count, 1);
                          lcd.print("(");
                          lcd.print(keybord[count]);
                          lcd.print(")");
                          name_k = 0;
                          break;
                        }
                      }
                      if (digitalRead(sw) == 0) {
                        while (digitalRead(sw) == 0)
                          ;
                        num[count_1] = keybord[count];
                        count = count_1;
                        count_o = -1;
                        break;
                      }
                    }
                  }
                }
                if (millis() - buttonPressTime < 2000) {
                  EEPROM.writeString(16, num);
                  count = 1;
                  count_o = -1;
                  break;
                }
              }
            }
          }
          if (digitalRead(sw) == 0 && count == 2) {
            while (digitalRead(sw) == 0 && count == 2)
              ;
            A_set = digitalRead(A);
            count_o = 1;
            int num = temp_p - 1;
            count = 50;
            while (true) {
              ky();
              if (count_o != count) {
                if (count < count_o) num--;
                else if (count > count_o) num++;
                if (num < 0) num = 0;
                if (num > Max_temp) num = Max_temp;
                lcd.setCursor(0, 2);
                lcd.print(" *");
                lcd.setCursor(8, 2);
                lcd.print(num);
                lcd.write(4);
                lcd.print("  ");
              }
              count_o = count;
              if (digitalRead(sw) == 0) {
                while (digitalRead(sw) == 0)
                  ;
                EEPROM.writeInt(27, num);
                temp_p = num;
                count = 2;
                break;
              }
            }
          }
          if (digitalRead(sw) == 0 && count == 3) {
            while (digitalRead(sw) == 0 && count == 3)
              ;
            A_set = digitalRead(A);
            count_o = 1;
            int num = motor_p - 1;
            count = 50;
            while (true) {
              ky();
              if (count_o != count) {
                if (count < count_o) num--;
                else if (count > count_o) num++;
                if (num < 0) num = 0;
                if (num > Max_temp) num = Max_temp;
                lcd.setCursor(0, 3);
                lcd.print(" *");
                lcd.setCursor(9, 3);
                lcd.print(num);
                lcd.print("%  ");
              }
              count_o = count;
              if (digitalRead(sw) == 0) {
                while (digitalRead(sw) == 0)
                  ;
                EEPROM.writeInt(31, num);
                motor_p = num;
                count = 3;
                break;
              }
            }
          }
        }
      }
    }
  }
}

void temperature_function() {
  ky();
  if (count < 0) count = 0;
  if (count > 2) count = 2;
  if (count_o != count) {
    if (count > 0) {
      lcd.setCursor(0, count - 1);
      lcd.print("  ");
    }
    if (count < 2) {
      lcd.setCursor(0, count + 1);
      lcd.print("  ");
    }
    lcd.setCursor(0, count);
    lcd.write(6);
    lcd.write(5);
    lcd.setCursor(2, 0);
    lcd.print("..   (Temperature)");
    lcd.setCursor(2, 1);
    lcd.print("Temp:");
    lcd.setCursor(13, 1);
    lcd.print("(");
    lcd.print(set_temp);
    lcd.print(")       ");
    lcd.setCursor(2, 2);
    lcd.print("Cooldown");
  }
  if (millis() - last_t1 > t1) {
    last_t1 = millis();
    lcd.setCursor(7, 1);
    lcd.print((int)temp);
    lcd.write(4);
    lcd.print(" ");
  }
  count_o = count;
  if (digitalRead(sw) == 0 && count == 0) {
    while (digitalRead(sw) == 0 && count == 0)
      ;
    lcd.clear();
    menu = -1;
    if (ready == 0) count = 2;
    else count = 3;
    A_set = digitalRead(A);
    count_o = -1;
    k_memu--;
  }
  if (digitalRead(sw) == 0 && count == 1) {
    while (digitalRead(sw) == 0 && count == 1)
      ;
    A_set = digitalRead(A);
    count_o = 1;
    int num = set_temp - 1;
    count = 50;
    while (true) {
      ky();
      if (count_o != count) {
        if (count < count_o) num--;
        else if (count > count_o) num++;
        if (num < 0) num = 0;
        if (num > Max_temp) num = Max_temp;
        lcd.setCursor(0, 1);
        lcd.print(" *");
        lcd.setCursor(2, 1);
        lcd.print("Temp:");
        lcd.setCursor(13, 1);
        lcd.print("(");
        lcd.print(num);
        lcd.print(")       ");
      }
      count_o = count;
      if (millis() - last_t1 > t1) {
        last_t1 = millis();
        lcd.setCursor(7, 1);
        lcd.print((int)temp);
        lcd.write(4);
        lcd.print(" ");
      }
      if (digitalRead(sw) == 0) {
        while (digitalRead(sw) == 0)
          ;
        set_temp = num;
        count = 1;
        break;
      }
    }
  }
  if (digitalRead(sw) == 0 && count == 2) {
    while (digitalRead(sw) == 0 && count == 2)
      ;
    set_temp = 0;
    lcd.setCursor(13, 1);
    lcd.print("(");
    lcd.print(set_temp);
    lcd.print(")       ");
  }
}

void motor_function() {
  ky();
  if (count < 0) count = 0;
  if (count > 3) count = 3;
  if (count_o != count) {
    if (count > 0) {
      lcd.setCursor(0, count - 1);
      lcd.print("  ");
    }
    if (count < 3) {
      lcd.setCursor(0, count + 1);
      lcd.print("  ");
    }
    lcd.setCursor(0, count);
    lcd.write(6);
    lcd.write(5);
    lcd.setCursor(2, 0);
    lcd.print("..         (Motor)");
    lcd.setCursor(2, 1);
    lcd.print("Speed:");
    lcd.print(set_speed);
    lcd.print("% ");
    lcd.setCursor(13, 1);
    lcd.print("(");
    lcd.print(set_speed);
    lcd.print(")   ");
    lcd.setCursor(2, 2);
    lcd.print("Start Motor");
    lcd.setCursor(2, 3);
    lcd.print("off Motor");
  }
  count_o = count;
  if (digitalRead(sw) == 0 && count == 0) {
    while (digitalRead(sw) == 0 && count == 0)
      ;
    lcd.clear();
    menu = -1;
    if (ready == 0) count = 3;
    else count = 4;
    A_set = digitalRead(A);
    count_o = -1;
    k_memu--;
  }
  if (digitalRead(sw) == 0 && count == 1) {
    while (digitalRead(sw) == 0 && count == 1)
      ;

    A_set = digitalRead(A);
    count_o = 1;
    int num = set_speed - 1;
    count = 50;
    while (true) {
      ky();
      if (count_o != count) {
        if (count < count_o) num--;
        else num++;
        if (num < 0) num = 0;
        if (num > Max_speed) num = Max_speed;
        lcd.setCursor(0, 1);
        lcd.print(" *");
        lcd.setCursor(2, 1);
        lcd.print("Speed:");
        lcd.print(set_speed);
        lcd.print("% ");
        lcd.setCursor(13, 1);
        lcd.print("(");
        lcd.print(num);
        lcd.print(")   ");
      }
      count_o = count;

      if (digitalRead(sw) == 0) {
        while (digitalRead(sw) == 0)
          ;
        set_speed = num;
        count = 1;
        break;
      }
    }
  }
  if (digitalRead(sw) == 0 && count == 2) {
    while (digitalRead(sw) == 0 && count == 2)
      ;
    Motor(set_speed);
  }
  if (digitalRead(sw) == 0 && count == 3) {
    while (digitalRead(sw) == 0 && count == 3)
      ;
    set_speed = 0;
    Motor(set_speed);
    lcd.setCursor(2, 1);
    lcd.print("Speed:");
    lcd.print(set_speed);
    lcd.print("% ");
    lcd.setCursor(13, 1);
    lcd.print("(");
    lcd.print(set_speed);
    lcd.print(")   ");
  }
}

void history_function() {
  ky();
  if (count < 0) count = 0;
  if (count > (ArraySize(history_menu) - 1)) count = (ArraySize(history_menu) - 1);
  if (count_o != count) {
    if (count > 0) {
      lcd.setCursor(0, count - 1);
      lcd.print("  ");
    }
    if (count < 2) {
      lcd.setCursor(0, count + 1);
      lcd.print("  ");
    }

    if (count < count_o) history_menu_en--;
    if (count > count_o) history_menu_en++;
    if (history_menu_en < 0) history_menu_en = 0;
    if (history_menu_en > 3) history_menu_en = 3;

    if (history_menu_en > 0) {
      lcd.setCursor(0, history_menu_en - 1);
      lcd.print("  ");
    }
    if (history_menu_en < 3) {
      lcd.setCursor(0, history_menu_en + 1);
      lcd.print("  ");
    }
    if (history_menu_en == 0) {
      lcd.setCursor(0, history_menu_en);
      lcd.write(6);
      lcd.write(5);
      lcd.setCursor(2, 0);
      lcd.print(history_menu[count]);
      lcd.setCursor(2, 1);
      lcd.print(history_menu[count + 1]);
      lcd.setCursor(2, 2);
      lcd.print(history_menu[count + 2]);
      lcd.setCursor(2, 3);
      lcd.print(history_menu[count + 3]);
    } else if (history_menu_en == 2) {
      lcd.setCursor(0, history_menu_en);
      lcd.write(6);
      lcd.write(5);
      lcd.setCursor(2, 0);
      lcd.print(history_menu[count - 2]);
      lcd.setCursor(2, 1);
      lcd.print(history_menu[count - 1]);
      lcd.setCursor(2, 2);
      lcd.print(history_menu[count]);
      lcd.setCursor(2, 3);
      lcd.print(history_menu[count + 1]);
    } else if (history_menu_en == 1) {
      lcd.setCursor(0, history_menu_en);
      lcd.write(6);
      lcd.write(5);
      lcd.setCursor(2, 0);
      lcd.print(history_menu[count - 1]);
      lcd.setCursor(2, 1);
      lcd.print(history_menu[count]);
      lcd.setCursor(2, 2);
      lcd.print(history_menu[count + 1]);
      lcd.setCursor(2, 3);
      lcd.print(history_menu[count + 2]);
    } else if (history_menu_en == 3) {
      lcd.setCursor(0, history_menu_en);
      lcd.write(6);
      lcd.write(5);
      lcd.setCursor(2, 0);
      lcd.print(history_menu[count - 3]);
      lcd.setCursor(2, 1);
      lcd.print(history_menu[count - 2]);
      lcd.setCursor(2, 2);
      lcd.print(history_menu[count - 1]);
      lcd.setCursor(2, 3);
      lcd.print(history_menu[count]);
    }
  }
  count_o = count;
  if (digitalRead(sw) == 0 && count == 0) {
    while (digitalRead(sw) == 0 && count == 0)
      ;
    lcd.clear();
    menu = -1;
    if (ready == 0) count = 4;
    else count = 5;
    A_set = digitalRead(A);
    count_o = -1;
    k_memu--;
  }
}

void setup_function() {
  ky();
  if (count < 0) count = 0;
  if (count > (ArraySize(setup_menu) - 1)) count = (ArraySize(setup_menu) - 1);
  if (count_o != count) {
    if (count < count_o) setup_memu--;
    if (count > count_o) setup_memu++;
    if (setup_memu < 0) setup_memu = 0;
    if (setup_memu > 3) setup_memu = 3;

    if (setup_memu > 0) {
      lcd.setCursor(0, setup_memu - 1);
      lcd.print("  ");
    }
    if (setup_memu < 3) {
      lcd.setCursor(0, setup_memu + 1);
      lcd.print("  ");
    }
    if (setup_memu == 0) {
      lcd.setCursor(0, setup_memu);
      lcd.write(6);
      lcd.write(5);
      lcd.setCursor(2, 0);
      lcd.print(setup_menu[count]);
      lcd.setCursor(2, 1);
      lcd.print(setup_menu[count + 1]);
      lcd.setCursor(2, 2);
      lcd.print(setup_menu[count + 2]);
      lcd.setCursor(2, 3);
      lcd.print(setup_menu[count + 3]);
    } else if (setup_memu == 2) {
      lcd.setCursor(0, setup_memu);
      lcd.write(6);
      lcd.write(5);
      lcd.setCursor(2, 0);
      lcd.print(setup_menu[count - 2]);
      lcd.setCursor(2, 1);
      lcd.print(setup_menu[count - 1]);
      lcd.setCursor(2, 2);
      lcd.print(setup_menu[count]);
      lcd.setCursor(2, 3);
      lcd.print(setup_menu[count + 1]);
    } else if (setup_memu == 1) {
      lcd.setCursor(0, setup_memu);
      lcd.write(6);
      lcd.write(5);
      lcd.setCursor(2, 0);
      lcd.print(setup_menu[count - 1]);
      lcd.setCursor(2, 1);
      lcd.print(setup_menu[count]);
      lcd.setCursor(2, 2);
      lcd.print(setup_menu[count + 1]);
      lcd.setCursor(2, 3);
      lcd.print(setup_menu[count + 2]);
    } else if (setup_memu == 3) {
      lcd.setCursor(0, setup_memu);
      lcd.write(6);
      lcd.write(5);
      lcd.setCursor(2, 0);
      lcd.print(setup_menu[count - 3]);
      lcd.setCursor(2, 1);
      lcd.print(setup_menu[count - 2]);
      lcd.setCursor(2, 2);
      lcd.print(setup_menu[count - 1]);
      lcd.setCursor(2, 3);
      lcd.print(setup_menu[count]);
    }
  }
  count_o = count;
  if (digitalRead(sw) == 0 && count == 0) {  // ..
    while (digitalRead(sw) == 0 && count == 0)
      ;
    lcd.clear();
    menu = -1;
    count = 5;
    A_set = digitalRead(A);
    count_o = -1;
    k_memu--;
  }
  if (digitalRead(sw) == 0 && count == 1) {  //Restore
    while (digitalRead(sw) == 0 && count == 1)
      ;
    lcd.clear();
    count = 1;
    count_o = -1;
    A_set = digitalRead(A);
    while (true) {
      ky();
      if (count < 1) count = 1;
      if (count > 2) count = 2;
      if (count_o != count) {
        if (count > 0) {
          lcd.setCursor(0, count - 1);
          lcd.print("  ");
        }
        if (count < 3) {
          lcd.setCursor(0, count + 1);
          lcd.print("  ");
        }
        lcd.setCursor(0, count);
        lcd.write(6);
        lcd.write(5);
        lcd.setCursor(2, 0);
        lcd.print("Restore Setting");
        lcd.setCursor(2, 1);
        lcd.print("Yes");
        lcd.setCursor(2, 2);
        lcd.print("No");
      }
      count_o = count;
      if (digitalRead(sw) == 0 && count == 1) {
        while (digitalRead(sw) == 0 && count == 1)
          ;
        lcd.clear();
        lcd.setCursor(2, 0);
        lcd.print("Restore Setting");
        lcd.setCursor(6, 2);
        lcd.print("Loading...");
        delay(2000);
        lcd.setCursor(6, 2);
        // data ที่จะเซฟ
        //
        lcd.print("  Done!   ");
        Buzzer(2);
        delay(1000);
        count = 1;
        A_set = digitalRead(A);
        count_o = -1;
        setup_memu--;
        break;
      }
      if (digitalRead(sw) == 0 && count == 2) {
        while (digitalRead(sw) == 0 && count == 2)
          ;
        lcd.clear();
        count = 1;
        A_set = digitalRead(A);
        count_o = -1;
        setup_memu--;
        break;
      }
    }
  }
  if (digitalRead(sw) == 0 && count == 2) {  //Pid Tuning
    while (digitalRead(sw) == 0 && count == 2)
      ;
    lcd.clear();
    count = 0;
    A_set = digitalRead(A);
    while (true) {
      ky();
      if (count < 0) count = 0;
      if (count > 3) count = 3;
      if (count_o != count) {
        if (count > 0) {
          lcd.setCursor(0, count - 1);
          lcd.print("  ");
        }
        if (count < 3) {
          lcd.setCursor(0, count + 1);
          lcd.print("  ");
        }
        lcd.setCursor(0, count);
        lcd.write(6);
        lcd.write(5);
        lcd.setCursor(2, 0);
        lcd.print("..           (PID)");
        lcd.setCursor(2, 1);
        lcd.print("Kp : ");
        lcd.print(Kp);
        lcd.print("  ");
        lcd.setCursor(2, 2);
        lcd.print("Ki : ");
        lcd.print(Ki);
        lcd.print("  ");
        lcd.setCursor(2, 3);
        lcd.print("Kd : ");
        lcd.print(Kd);
        lcd.print("  ");
      }
      count_o = count;
      if (digitalRead(sw) == 0 && count == 0) {
        while (digitalRead(sw) == 0 && count == 0)
          ;
        lcd.clear();
        count = 2;
        A_set = digitalRead(A);
        count_o = -1;
        setup_memu--;
        break;
      }
    }
  }

  if (digitalRead(sw) == 0 && count == 3) {  //Encoder Tuning
    while (digitalRead(sw) == 0 && count == 3)
      ;
    lcd.clear();
    count = 0;
    A_set = digitalRead(A);
    while (true) {
      ky();
      if (count < 0) count = 0;
      if (count > 3) count = 3;
      if (count_o != count) {
        if (count > 0) {
          lcd.setCursor(0, count - 1);
          lcd.print("  ");
        }
        if (count < 3) {
          lcd.setCursor(0, count + 1);
          lcd.print("  ");
        }
        lcd.setCursor(0, count);
        lcd.write(6);
        lcd.write(5);
        lcd.setCursor(2, 0);
        lcd.print("..       (Encoder)");
        lcd.setCursor(2, 1);
        lcd.print("Clear");
      }
      count_o = count;
      if (digitalRead(sw) == 0 && count == 0) {
        while (digitalRead(sw) == 0 && count == 0)
          ;
        lcd.clear();
        count = 3;
        A_set = digitalRead(A);
        count_o = -1;
        setup_memu--;
        break;
      }
      if (digitalRead(sw) == 0 && count == 1) {
        while (digitalRead(sw) == 0 && count == 1)
          ;
        _Ticks = 0.0;
        Buzzer(2);
      }
    }
  }

  if (digitalRead(sw) == 0 && count == 4) {  //Proximity
    while (digitalRead(sw) == 0 && count == 4)
      ;
    lcd.clear();
    count = 0;
    A_set = digitalRead(A);
    while (true) {
      ky();
      if (count < 0) count = 0;
      if (count > 1) count = 1;
      if (count_o != count) {
        if (count > 0) {
          lcd.setCursor(0, count - 1);
          lcd.print("  ");
        }
        if (count < 3) {
          lcd.setCursor(0, count + 1);
          lcd.print("  ");
        }
        lcd.setCursor(0, count);
        lcd.write(6);
        lcd.write(5);
        lcd.setCursor(2, 0);
        lcd.print("..     (Proximity)");
        lcd.setCursor(2, 1);
        lcd.print("Status: (");
      }
      if (millis() - last_prox > prox) {
        last_prox = millis();
        lcd.setCursor(11, 1);
        lcd.print(proximity);
        lcd.print(")");
      }
      count_o = count;
      if (digitalRead(sw) == 0 && count == 0) {
        while (digitalRead(sw) == 0 && count == 0)
          ;
        lcd.clear();
        count = 4;
        A_set = digitalRead(A);
        count_o = -1;
        setup_memu--;
        break;
      }
    }
  }

  if (digitalRead(sw) == 0 && count == 5) {  //Buzzer
    while (digitalRead(sw) == 0 && count == 5)
      ;
    lcd.clear();
    count = 0;
    count_o = -1;
    A_set = digitalRead(A);
    while (true) {
      ky();
      if (count < 0) count = 0;
      if (count > 1) count = 1;
      if (count_o != count) {
        if (count > 0) {
          lcd.setCursor(0, count - 1);
          lcd.print("  ");
        }
        if (count < 1) {
          lcd.setCursor(0, count + 1);
          lcd.print("  ");
        }
        lcd.setCursor(0, count);
        lcd.write(6);
        lcd.write(5);
        lcd.setCursor(2, 0);
        lcd.print("..        (Buzzer)");
        lcd.setCursor(2, 1);
        if (buzzer_on_off == 0) lcd.print("On Buzzer ");
        else lcd.print("Off Buzzer");
      }
      count_o = count;
      if (digitalRead(sw) == 0 && count == 0) {
        while (digitalRead(sw) == 0 && count == 0)
          ;
        lcd.clear();
        count = 5;
        A_set = digitalRead(A);
        count_o = -1;
        setup_memu--;
        break;
      }
      if (digitalRead(sw) == 0 && count == 1) {
        while (digitalRead(sw) == 0 && count == 1)
          ;
        if (buzzer_on_off == 0) {
          buzzer_on_off = 1;
          lcd.setCursor(2, 1);
          lcd.print("Off Buzzer");
        } else {
          buzzer_on_off = 0;
          lcd.setCursor(2, 1);
          lcd.print("On Buzzer ");
          Buzzer(2);
        }
      }
    }
  }
}

void save_config_function() {
  ky();
  if (count <= 0) count = 1;
  if (count > 2) count = 2;
  if (count_o != count) {
    if (count > 0) {
      lcd.setCursor(0, count - 1);
      lcd.print("  ");
    }
    if (count < 3) {
      lcd.setCursor(0, count + 1);
      lcd.print("  ");
    }
    lcd.setCursor(0, count);
    lcd.write(6);
    lcd.write(5);
    lcd.setCursor(2, 0);
    lcd.print("Save config");
    lcd.setCursor(2, 1);
    lcd.print("Yes");
    lcd.setCursor(2, 2);
    lcd.print("No");
  }
  count_o = count;
  if (digitalRead(sw) == 0 && count == 1) {
    while (digitalRead(sw) == 0 && count == 1)
      ;
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print("Save config");
    lcd.setCursor(6, 2);
    lcd.print("Loading...");
    delay(2000);
    lcd.setCursor(6, 2);
    // data ที่จะเซฟ
    //
    lcd.print("  Done!   ");
    Buzzer(2);
    delay(1000);

    EEPROM.commit();  // eeprom
    ESP.restart();
  }
  if (digitalRead(sw) == 0 && count == 2) {
    while (digitalRead(sw) == 0 && count == 2)
      ;
    lcd.clear();
    count = 6;
    A_set = digitalRead(A);
    count_o = -1;
    k_memu--;
    menu = -1;
  }
}

double analogtemp() {
  /* Original Code
  R1 = 10000.0;   // voltage divider resistor value 10k
  Beta = 3950.0;  // Beta value
  To = 298.15;    // Temperature in Kelvin for 25 degree Celsius
  Ro = 100000.0;   // Resistance of Thermistor at 25 degree Celsius
  adcMax = 4095;  // ADC resolution 12-bit (0-4095)
  Vref = 3.3;     // Vref

  Vout = adc * Vs/adcMax;
  Rt = R1 * Vout / (Vs - Vout);
  T = 1/(1/To + log(Rt/Ro)/Beta);    // Temperature in Kelvin
  Tc = T - 273.15;                   // Celsius (C)  */

  //230-250 4050
  //210-230 4010
  //185-210 3980

  int16_t adc = ads.readADC_SingleEnded(0);
  double Vout = adc * 4.98 / 26559;                         // Adcmax 32767/6.144V*4.94V = 26345
  double Rt = 9760.0 * Vout / (4.98 - Vout);                // resistor value 10k วัดได้ 9.76k
  double T = 1 / (1 / 298.15 + log(Rt / 100000.0) / Beta);  // Temperature in Kelvin
  double Tc = T - 273.15;                                   // Celsius (C)  */
  return Tc;
}

void Pid_Temp() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= sampleTime) {
    if (temp >= Chamber_temp && set_temp > 0 && temp <= Max_temp) {
      float dt = (currentTime - lastTime) / 1000.0;  // เวลาในหน่วยวินาที

      if (set_temp <= 185) Beta = 3950.0;
      else if (set_temp <= 210) Beta = 3980.0;
      else if (set_temp <= 230) Beta = 4010.0;
      else Beta = 4050.0;

      temp = analogtemp();
      error = set_temp - temp;
      integral += error * dt;
      derivative = (error - pre_error) / dt;

      int Ki_cal = Ki * integral;
      Ki_cal = constrain(Ki_cal, 0, 255);

      PID_Val = (Kp * error) + Ki_cal + (Kd * derivative);

      PID_Val = constrain(PID_Val, 0, 255);
      ledcWrite(pid_Ch, PID_Val);

      pre_error = error;
    } else {
      set_temp = 0;
      PID_Val = 0;
      temp = analogtemp();
      ledcWrite(pid_Ch, 0);

      //set_temp = 190;
    }
    lastTime = currentTime;
  }
}

void Motor(int i) {
  int i_speed = map(i, 0, Max_speed, 0, 255);
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWM_Motor, i_speed);
}

void sc_time() {  // พักหน้าจอ
  if (menu != 0) {
    if (count_o != count) count_time = 0;
    if (millis() - last_t3 > t3) {
      last_t3 = millis();
      count_time++;
    }
    if (count_time == time_t3) {
      lcd.clear();
      j = 0;
      menu = 0;
      count = 0;
      A_set = digitalRead(A);
      count_o = -1;
      count_time = 0;
    }
  }
}

void ky() {
  int A_val = digitalRead(A);
  if (A_val != A_set) {
    if (digitalRead(B) == A_val) count++;
    else count--;
  }
  A_set = A_val;
}

void Buzzer(int B_i) {
  if (buzzer_on_off == 0) {
    if (B_i == 1) {
      ledcWrite(buzzer_ch, 50);
      delay(80);
      ledcWrite(buzzer_ch, 0);
    } else {
      ledcWrite(buzzer_ch, 50);
      delay(80);
      ledcWrite(buzzer_ch, 0);
      delay(50);
      ledcWrite(buzzer_ch, 50);
      delay(80);
      ledcWrite(buzzer_ch, 0);
    }
  }
}

void Buzzer_proximity() {
  if (proximity_check == 1 && proximity != 1) {
    alarmBuzzer(1000);
    undetected = true;

    if (ff == 0) {
      // lcd.clear();
      // delay(80);

      ready_menu[1] = "Stop alarm      ";
      Motor(0);
      start_stop = 0;

      j = 0;
      count = 0;
      A_set = digitalRead(A);
      count_o = -1;
      ff = 1;
      menu = 0;
    }
  } else if (proximity_check == 1 && proximity == 1) {
    ledcWrite(buzzer_ch, 0);
    undetected = false;
  }
}

void alarmBuzzer(unsigned long duration) {
  while (millis() - last_alarmBuzzer > duration) {
    last_alarmBuzzer = millis();
    if (!buzzerActive) {
      ledcWrite(buzzer_ch, 50);
      buzzerActive = true;
      break;
    } else {
      ledcWrite(buzzer_ch, 0);
      buzzerActive = false;
      break;
    }
  }
}

void Proximity() {
  proximity = digitalRead(4);
}

String floattostring(float num, int mode) {
  int Ex_Int = int(num);               // สร้างตัวแปรชื่อ Ex_Int ชนิด int (จำนวนเต็ม) จากนั้นแปลง Ex_Float เป็นจำนวนเต็ม ทำให้ทศนิยมถูกตัดออก
  long Ex_Dec = 100 * (num - Ex_Int);  // สร้างตัวแปรชื่อ Ex_Int ชนิด int (จำนวนเต็ม)
  String name = String(Ex_Int) + "." + String(Ex_Dec);
  int number = 18 - name.length();

  for (int i = 0; i < number; i++) {
    if (i == 0) name += "m";
    if (i != 0 && mode == 0) name += " ";

    int j = number - 9;
    if (i != 0 && i <= j && mode != 0) name += " ";
  }
  if (mode != 0) name += "(recent)";
  return name;
}

String name_eeprom(String name) {
  for (int i = name.length(); i < 15; i++) {
    name += " ";
  }
  name += ">  ";
  return name;
}

void Status_memu() {
  if (temp >= Chamber_temp && set_temp == 0 && temp <= 40) {
    lcd.setCursor(0, 3);
    lcd.print("Ready           ");
  } else if (temp < Chamber_temp && temp < Max_temp) {
    lcd.setCursor(0, 3);
    lcd.print("Temperature fail");
  } else if (temp > Max_temp) {
    lcd.setCursor(0, 3);
    lcd.print("Temperature over");
  } else if (undetected) {
    lcd.setCursor(0, 3);
    lcd.print("Prox undetected");
  } else if (set_temp > 0) {
    lcd.setCursor(0, 3);
    lcd.print("Heating(");
    lcd.print((int)PID_Val);
    lcd.print(")     ");
  } else if (temp > 40 && temp < Max_temp) {
    lcd.setCursor(0, 3);
    lcd.print("Cooldown        ");
  }
}