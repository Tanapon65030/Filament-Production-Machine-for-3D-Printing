TaskHandle_t Task1 = NULL;
TaskHandle_t Task2 = NULL;
hw_timer_t* timer = NULL;
int t = 1;

//temp
#define NTC 25
const int temp_pin = 33;
const int pid_Ch = 0;

int set_temp = 195;
int Max_temp = 250;
double Kp = 20;
double Ki = 0.1;
double Kd = 60;
double temp, PID_Val, error, pre_error, sum_error;

//Motor
#define STBY 17     //standby
#define PWM_pin 19  //Speed control
#define AIN1 18     //Direction
#define AIN2 5      //Direction
const int PWM_Motor = 1;
int set_speed = 80;
int Max_speed = 100;
int jk = 0;

void IRAM_ATTR onTimer() {
  Pid_Temp();
}

void setup() {
  Serial.begin(115200);

  //temp
  ledcSetup(pid_Ch, 5000, 8);
  ledcAttachPin(temp_pin, pid_Ch);

  //Motor
  ledcSetup(PWM_Motor, 5000, 8);
  ledcAttachPin(PWM_pin, PWM_Motor);
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 0, &Task1, 0);
  delay(500);
  xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 2, &Task2, 1);
  delay(500);
}

void Task1code(void* pvParameters) {
  timer = timerBegin(0, 80, true);              // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer, &onTimer, true);  // edge (not level) triggered
  timerAlarmWrite(timer, (1000000 * t), true);  // 1000000 * 1 us = 1 s, autoreload true
  timerAlarmEnable(timer);                      // enable

  for (;;) {
    temp = analogtemp();
    // Serial.print(count);
    // Serial.print("\t    j: ");
    // Serial.print(j);
    // Serial.print("\t    k_memu: ");
    // Serial.print(k_memu);
    // Serial.print("\t    count_time: ");
    // Serial.println(count_time);
  }
}

void Task2code(void* pvParameters) {
  pinMode(14, INPUT_PULLUP);
  for (;;) {
    Serial.print("\t    PID: ");
    Serial.print(PID_Val);
    Serial.print("\t    temp: ");
    Serial.println(temp);


    if (digitalRead(14) == 0) {
      while (digitalRead(14) == 0)
        ;
      if (jk == 0) {
        Motor(set_speed);
        jk = 1;
      } else {
        Motor(0);
        jk = 0;
      }
    }
  }
}

void loop() {}

double analogtemp() {
  /* Original Code
  R1 = 10000.0;   // voltage divider resistor value 10k
  Beta = 3950.0;  // Beta value
  To = 298.15;    // Temperature in Kelvin for 25 degree Celsius
  Ro = 10000.0;   // Resistance of Thermistor at 25 degree Celsius
  adcMax = 4095;  // ADC resolution 12-bit (0-4095)
  Vref = 3.3;     // Vref

  Vout = adc * Vs/adcMax;
  Rt = R1 * Vout / (Vs - Vout);
  T = 1/(1/To + log(Rt/Ro)/Beta);    // Temperature in Kelvin
  Tc = T - 273.15;                   // Celsius (C)  */

  double Vout = analogRead(NTC) * 3.3 / 4095;
  double Rt = 10000.0 * Vout / (3.3 - Vout);
  double Tc = (1 / (1 / 298.15 + log(Rt / 100000.0) / 3950)) - 273.15;
  return Tc;
}

void Pid_Temp() {
  error = set_temp - temp;
  PID_Val = (Kp * error) + (Ki * (sum_error)) + (Kd * ((error - pre_error) / t));

  if (PID_Val > 255) PID_Val = 255;
  if (PID_Val < 0) PID_Val = 0;

  pre_error = error;
  sum_error += error;
  ledcWrite(pid_Ch, PID_Val);
}

void Motor(int i) {
  int i_speed = map(i, 0, Max_speed, 0, 255);
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWM_Motor, i_speed);
}
