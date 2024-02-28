#include <driver/pcnt.h>
TaskHandle_t Task1 = NULL;
TaskHandle_t Task2 = NULL;
/* create a hardware timer */
hw_timer_t* timer = NULL;
double _Ticks = 0.0;

#define _pinA 13
#define _pinB 12

void IRAM_ATTR Update_Ticks() {
  int16_t c;
  pcnt_get_counter_value(PCNT_UNIT_0, &c);  //Get pulse counter value.
  if ((_Ticks + c) != _Ticks) {
    pcnt_counter_clear(PCNT_UNIT_0);                 //Clear and reset PCNT counter value to zero.
    double radius_en = 2 * 3.1416 * 16;              //ความยาวรอบวงของแกนที่หมุน = 2 × π × รัศมีของวงกลม(16mm)
    double rpm_en = c / 400.0;                       //จำนวนรอบ = จำนวน pulses ที่ได้รับ / 200 pulses/รอบ **จับทั้ง AB 400 pulses/รอบ
    double lenth_en = (rpm_en * radius_en) * 0.001;  //ความยาวที่วัดได้ = จำนวนรอบ × ความยาวรอบวงของแกนที่หมุน **แปลงหน่วยเป็นเมตร
    _Ticks = _Ticks + lenth_en;                      //อัพเดตความยาว
  }
}

void setup() {
  Serial.begin(115200);

  xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 0, &Task1, 0);
  delay(500);
  xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 2, &Task2, 1);
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
    .pos_mode = PCNT_COUNT_DEC,       //PCNT positive edge count mode, Decrease counter value  //PCNT_COUNT_DIS
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
    Serial.println(_Ticks);
  }
}

void Task2code(void* pvParameters) {
  for (;;) {
  }
}

void loop() {}
