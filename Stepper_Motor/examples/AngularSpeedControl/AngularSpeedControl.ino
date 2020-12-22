#define pinSTEP 3                 // пин, подключённый к STEP драйвера
#define pinDIR 2                  // пин, подключённый к DIR драйвера
#define microstep 16              // деление шага
#define Delay 1000                // время задержки
#include "Stepper_Motor.h"        // подключаем библиотеку

Stepper_Motor Motor1(pinSTEP, pinDIR, microstep); // создаём объект мотора

void setup() {
  Serial.begin(9600);                       // начинаем связь через Serial порт
  Serial.println(Motor1.getAngularPosition());  // Получаем начальный угол в абсолютной системе

  //Motor1.setGearRatio(1);                 // установка передаточного отношения (ratio > 1 - редуктор; 0 < ratio < 1 - мультипликатор)
  //Motor1.setDirection(0);                 // установка направления движения мотора (по умолчанию - 0)
  //Motor1.setAngularSpeed(1);              // по умолчанию скорость 1 об/с
  //Motor1.setAngularAcceleration(2);       // по умолчанию ускорение 2 об/с^2

  // перемещение в абсолютной системе координат
  // /*
  Motor1.setCoordinateSystem(0);            // устанавливаем систему координат (по умолчанию - абсолютная)
  Motor1.goAngularPosition(360*2);          // перемещение в позицию 720 градусов
  delay(Delay);                             // задержка

  Motor1.setAngularSpeed(2);                // установка скорости 2 об/сек
  Motor1.goAngularPosition(360*4);          // перемещение в позицию 1440 градусов
  delay(Delay);                             // задержка

  Motor1.setAngularSpeed(3);                // установка скорости 3 об/сек
  Motor1.goAngularPosition(360*6);          // перемещение в позицию 2160 градусов
  delay(Delay);                             // задержка

  Motor1.setAngularSpeed(4);                // установка скорости 4 об/сек
  Motor1.goAngularPosition(360*8);          // перемещение в позицию 2880 градусов
  delay(Delay);                             // задержка

  Motor1.setAngularSpeed(5);                // установка скорости 5 об/сек
  Motor1.goAngularPosition(0);              // перемещение в позицию 0 градусов
  Serial.println(Motor1.getAngularPosition());  // Получаем конечный угол в абсолютной системе
  // */

  //перемещение в относительной системе координат
  /*
  Motor1.setCoordinateSystem(1);            // устанавливаем систему координат (по умолчанию - абсолютная)
  Motor1.goAngularPosition(360*2);          // перемещение на 720 градусов
  delay(Delay);                             // задержка

  Motor1.setAngularSpeed(2);                // установка скорости 2 об/сек
  Motor1.goAngularPosition(360*2);          // перемещение на 720 градусов
  delay(Delay);                             // задержка

  Motor1.setAngularSpeed(3);                // установка скорости 3 об/сек
  Motor1.goAngularPosition(360*2);          // перемещение на 720 градусов
  delay(Delay);                             // задержка

  Motor1.setAngularSpeed(4);                // установка скорости 4 об/сек
  Motor1.goAngularPosition(360*2);          // перемещение на 720 градусов
  delay(Delay);                             // задержка

  Motor1.setAngularSpeed(5);                // установка скорости 5 об/сек
  Motor1.goAngularPosition(-2880);          // перемещение в позицию 0 градусов
  Serial.println(Motor1.getAngularPosition());  // Получаем конечный угол в абсолютной системе
  */
}

void loop() {
  // put your main code here, to run repeatedly:

}
