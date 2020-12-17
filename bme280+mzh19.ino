#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1041.25)
Adafruit_BME280 bme;

#define pwmPin 5
#define LedPin 13
#include <LiquidCrystal_I2C.h> // подключаем библиотеку для QAPASS 1602
LiquidCrystal_I2C LCD(0x27,16,2); // присваиваем имя LCD для дисплея
int prevVal = LOW;
long th, tl, h, l, ppm, diff;

void setup() {
  Serial.begin(9600);
   pinMode(pwmPin, INPUT);
  pinMode(LedPin, OUTPUT);

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
 

  // LCD.init(); // инициализация LCD дисплея
  // LCD.backlight(); // включение подсветки дисплея
   
  // LCD.setCursor(0,0);     // ставим курсор на 1 символ первой строки
  // LCD.print("PPM: ");     // печатаем сообщение на первой строке

  // LCD.setCursor(0,1);     // ставим курсор на 1 символ первой строки
 //  LCD.print("DO HOPMbI: ");     // печатаем сообщение на первой строке
  
}

void loop() {

  long tt = millis();
  int myVal = digitalRead(pwmPin);

    //Если обнаружили изменение
  if (myVal == HIGH) {
    digitalWrite(LedPin, HIGH);
    if (myVal != prevVal) {
      h = tt;
      tl = h - l;
      prevVal = myVal;
    }
  }  else {
    digitalWrite(LedPin, LOW);
    if (myVal != prevVal) {
      l = tt;
      th = l - h;
      prevVal = myVal;
      ppm = 5000 * (th - 2) / (th + tl - 4);

      //delay(6000);
      Serial.println("PPM = " + String(ppm));
      
      Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println("*C");

  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() *0.007500637554192,2);
  Serial.println("mmHg");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println("m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println("%");

  Serial.println();
  //delay(5000);
    }
  } 
  

  
  //delay(1000);

  

 //  LCD.setCursor(13,0);     // ставим курсор на 1 символ первой строки
  // LCD.print(ppm);     // печатаем сообщение на первой строке

  // LCD.setCursor(13,1);     // ставим курсор на 1 символ первой строки
  // LCD.print(diff);     // печатаем сообщение на первой строке
  
}
