#include <OneWire.h>
#include <DallasTemperature.h>
#include <Ticker.h>
#include <Arduino.h>  // Necessário para utilizar FreeRTOS no ESP32
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define TEMP_PIN 4
#define PWM_PIN 5

#define BUTTON_UP 13
#define BUTTON_DOWN 14

LiquidCrystal_I2C lcd(0x38, 16, 2);

unsigned long up_press = 0;
unsigned long down_press = 0;
bool status = false;

const int freq = 1000;    // Frequency (in Hz)
const int resolution = 8; // Resolution of PWM (8 bits)
const int channel = 0;    // PWM channel

Ticker read_sensor_timer;
Ticker controller_timer;
Ticker display_timer;

OneWire oneWire(TEMP_PIN);
DallasTemperature temp_sensor(&oneWire);

// Constantes baseadas na função de transferência
// const float a1 = 0.9998;
const float b0 = 0.007816;
const float b1 = 0.007816;

// Variáveis para armazenar os valores atuais e anteriores
float y_current = 0;
float y_previous = 0;
float u_current = 0;
float u_previous = 0;
float y_controller = 0.0;
float y_max = 20;
float y_min = 0;

// Set Point desejado (em graus Celsius)
float setPoint = 40.0; 
float lastSetpoint = 40.0;

float temp = 0.0;
int duty = 0;

// Função que realiza a leitura da temperatura
void read_sensor() {
    // Inicia a conversão de temperatura, não espera pela conclusão
    // temp_sensor.setWaitForConversion(false);
  temp_sensor.requestTemperatures();
  temp = temp_sensor.getTempCByIndex(0);
}

// Função que realiza o cálculo do controlador
void calculate_control() {
  if(status == true) {
    u_current = setPoint - temp;
    y_current = b0 * u_current + b1 * u_previous;
    u_previous = u_current;
    y_previous = y_current;
    y_controller = y_current;
    set_pwm();
  } else {
    return;
  }

}

void update_display() {
  lcd.clear();
  lcd.setCursor(0, 0);
  if(status == true){
    lcd.printf("Temp: %.2f C", temp);
    lcd.setCursor(0, 1);
    lcd.printf("Set: %.2f C", setPoint);
  } else {
    lcd.printf("DESLIGADO");
  }
}

void turn_off() {
  ledcWrite(PWM_PIN, 0);
}

// Função que ajusta o PWM
void set_pwm() {
  if (y_controller > y_max) {
    y_controller = y_max;
  } else if (y_controller < y_min) {
    y_controller = y_min;
  }
  
  int pwm = map(y_controller * 100, y_min, y_max, 0, 255);
  duty = map(pwm, 0, 255, 0, 100);
  if(pwm >= 128){
    pwm = 128;
  }

  ledcWrite(PWM_PIN, pwm);
}

bool both_pressed(unsigned long first, unsigned long second) {
  if(second - first <= 1000){
    return true;
  } else {
    return false; 
  }
}

void set_power(){
  setPoint = lastSetpoint;
  status = !status;
  if(status == false){
    turn_off();
    
  }
}

void IRAM_ATTR up_ISR() {
  up_press = millis();
  if(both_pressed(down_press, up_press)){
    set_power();
    return;
  } else if(status == true) {
    lastSetpoint = setPoint;
     if(setPoint < 60) {
    setPoint += 2.0;
  }
  }
}

void IRAM_ATTR down_ISR() {
  down_press = millis();
  if(both_pressed(up_press, down_press)){
    set_power();
    return;
  } else if(status == true) {
    lastSetpoint = setPoint;
     if(setPoint > 40) {
    setPoint -= 2.0;
  }
  }
}

void setup() {
  Serial.begin(115200);
  temp_sensor.begin();
  
  // Configuração do PWM
  ledcAttach(PWM_PIN, freq, resolution);

  // Configuração do timer para leitura do sensor
  read_sensor_timer.attach(0.2, read_sensor);
  controller_timer.attach(0.1, calculate_control);
  display_timer.attach(1, update_display);

  lcd.init();
  lcd.backlight();

  pinMode(BUTTON_UP, INPUT);
  attachInterrupt(BUTTON_UP, up_ISR, RISING);

  pinMode(BUTTON_DOWN, INPUT);
  attachInterrupt(BUTTON_DOWN, down_ISR, RISING);
}

void loop() {
  Serial.printf("Temperatura: %.2f\n", temp);
  Serial.printf("Controlador: %.2f\n", y_controller);
  Serial.printf("Duty: %d \n", duty);
  Serial.printf("Setpoint: %.2f \n", setPoint);
  Serial.printf("Status: %d \n", status);
  delay(2000);
}
