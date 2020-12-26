#define pinSTEP 3                 // пин, подключённый к STEP драйвера
#define pinDIR 2                  // пин, подключённый к DIR драйвера
#define microstep 16              // деление шага
#define Delay 1000                // время задержки
#include "Stepper_Motor.h"        // подключаем библиотеку

Stepper_Motor Motor1(pinSTEP, pinDIR, microstep); // создаём объект мотора

void setup() {
  Serial.begin(9600);                       // начинаем связь через Serial порт
  Serial.println(Motor1.getLinearPosition());  // Получаем начальный угол в абсолютной системе

  //Motor1.setGearRatio(1);                 // установка передаточного отношения (ratio > 1 - редуктор; 0 < ratio < 1 - мультипликатор)
  //Motor1.setDirection(0);                 // установка направления движения мотора (по умолчанию - 0)
  //Motor1.setLinearSpeed(1);               // по умолчанию скорость 1 мм/с
  //Motor1.setLinearAcceleration(2);        // по умолчанию ускорение 2 мм/с^2

  // перемещение в абсолютной системе координат
  // /*
  Motor1.setCoordinateSystem(0);            // устанавливаем систему координат (по умолчанию - абсолютная)
  Motor1.setLinearAcceleration(0.2);        // установка ускорения 0.2 мм/сек^2
  Motor1.goLinearPosition(2);               // перемещение в позицию 2 мм
  delay(Delay);                             // задержка

  Motor1.setLinearAcceleration(0.5);        // установка ускорения 0.5 мм/сек^2
  Motor1.goLinearPosition(4);               // перемещение в позицию 4 мм
  delay(Delay);                             // задержка

  Motor1.setLinearAcceleration(1);          // установка ускорения 1 мм/сек^2
  Motor1.goLinearPosition(6);               // перемещение в позицию 6 мм
  delay(Delay);                             // задержка

  Motor1.setLinearAcceleration(2);          // установка ускорения 2 мм/сек^2
  Motor1.goLinearPosition(8);               // перемещение в позицию 8 мм
  delay(Delay);                             // задержка

  Motor1.setLinearAcceleration(4);          // установка ускорения 4 мм/сек^2
  Motor1.goLinearPosition(0);               // перемещение в позицию 0 мм
  Serial.println(Motor1.getLinearPosition());  // Получаем конечный угол в абсолютной системе
  // */

  //перемещение в относительной системе координат
  /*
  Motor1.setCoordinateSystem(1);            // устанавливаем систему координат (по умолчанию - абсолютная)
  Motor1.setLinearAcceleration(0.2);        // установка ускорения 0.2 мм/сек^2
  Motor1.goLinearPosition(2);               // перемещение на 2 мм
  delay(Delay);                             // задержка

  Motor1.setLinearAcceleration(0.5);        // установка ускорения 0.5 мм/сек^2
  Motor1.goLinearPosition(2);               // перемещение на 2 мм
  delay(Delay);                             // задержка

  Motor1.setLinearAcceleration(1);          // установка ускорения 1 мм/сек^2
  Motor1.goLinearPosition(2);               // перемещение на 2 мм
  delay(Delay);                             // задержка

  Motor1.setLinearAcceleration(2);          // установка ускорения 2 мм/сек^2
  Motor1.goLinearPosition(2);               // перемещение на 2 мм
  delay(Delay);                             // задержка

  Motor1.setLinearAcceleration(4);          // установка ускорения 4 мм/сек^2
  Motor1.goLinearPosition(-8);              // перемещение на -8 мм
  Serial.println(Motor1.getLinearPosition());  // Получаем конечный угол в абсолютной системе
  */
}

void loop() {
  // put your main code here, to run repeatedly:

}
