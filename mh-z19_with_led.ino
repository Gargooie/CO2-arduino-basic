//requires display refresh fix
// pins a4,a5 for display, d5 for mhz19

#define pwmPin 5
#define LedPin 13

#include <Wire.h> // библиотека для управления устройствами по I2C 
#include <LiquidCrystal_I2C.h> // подключаем библиотеку для QAPASS 1602

LiquidCrystal_I2C LCD(0x27,16,2); // присваиваем имя LCD для дисплея

int prevVal = LOW;
long th, tl, h, l, ppm, diff;

void setup() {
  Serial.begin(9600);
  pinMode(pwmPin, INPUT);
  pinMode(LedPin, OUTPUT);


   LCD.init(); // инициализация LCD дисплея
   LCD.backlight(); // включение подсветки дисплея
   
   LCD.setCursor(0,0);     // ставим курсор на 1 символ первой строки
   LCD.print("PPM: ");     // печатаем сообщение на первой строке

   LCD.setCursor(0,1);     // ставим курсор на 1 символ первой строки
   LCD.print("DO HOPMbI: ");     // печатаем сообщение на первой строке
       
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
      diff = ppm-600;

      //delay(60000);
      Serial.println("PPM = " + String(ppm));
    }
  }

   LCD.setCursor(13,0);     // ставим курсор на 1 символ первой строки
   LCD.print(ppm);     // печатаем сообщение на первой строке

   LCD.setCursor(13,1);     // ставим курсор на 1 символ первой строки
   LCD.print(diff);     // печатаем сообщение на первой строке
   

}
