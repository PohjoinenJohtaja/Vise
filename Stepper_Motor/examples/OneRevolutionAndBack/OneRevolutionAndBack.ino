#define pinSTEP 3                 // пин, подключённый к STEP драйвера
#define pinDIR 2                  // пин, подключённый к DIR драйвера
#define microstep 16              // деление шага
#define Delay 1000                // время задержки
#include "Stepper_Motor.h"        // подключаем библиотеку

Stepper_Motor Motor1(pinSTEP, pinDIR, microstep); // создаём объект мотора

void setup() {
  Serial.begin(9600);                       // начинаем связь через Serial порт
  Serial.println(Motor1.getAngularPosition());  // Получаем начальный угол в абсолютной системе

  //Motor1.setDirection(0);                 // установка направления движения мотора (по умолчанию - 0)
  //Motor1.setAngularSpeed(1);              // по умолчанию скорость 1 об/с
  //Motor1.setAngularAcceleration(2);       // по умолчанию ускорение 2 об/с^2

  // перемещение в абсолютной системе координат
  // /*
  Motor1.setCoordinateSystem(0);            // устанавливаем систему координат (по умолчанию - абсолютная)
  Motor1.goAngularPosition(360);            // перемещение в позицию 360 градусов
  delay(Delay);                             // задержка
  Motor1.goAngularPosition(0);              // перемещение в позицию 0 градусов
  Serial.println(Motor1.getAngularPosition());  // Получаем конечный угол в абсолютной системе
  // */

  //перемещение в относительной системе координат
  /*
  Motor1.setCoordinateSystem(1);            // устанавливаем систему координат (по умолчанию - абсолютная)
  Motor1.goAngularPosition(360);            // перемещение на 360 градусов
  delay(Delay);                             // задержка
  Motor1.goAngularPosition(-360);           // перемещение на -360 градусов
  Serial.println(Motor1.getAngularPosition());  // Получаем конечный угол в абсолютной системе
  */
}

void loop() {
  // put your main code here, to run repeatedly:

}
