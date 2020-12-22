#define pinSTEP 3                 // пин, подключённый к STEP драйвера
#define pinDIR 2                  // пин, подключённый к DIR драйвера
#define microstep 16              // деление шага
#define Delay 1000                // время задержки
#include "Stepper_Motor.h"        // подключаем библиотеку

Stepper_Motor Motor1(pinSTEP, pinDIR, microstep); // создаём объект мотора

void setup() {
  Serial.begin(9600);                       // начинаем связь через Serial порт
  Serial.println(Motor1.getLinearPosition());  // Получаем начальную позицию в абсолютной системе

  //Motor1.setGearRatio(1);                 // установка передаточного отношения (ratio > 1 - редуктор; 0 < ratio < 1 - мультипликатор)
  Motor1.setThreadPitch(2);               // установка шага винта (мм) или диаметра шкива (по умолчанию - 1)
  //Motor1.setDirection(0);                 // установка направления движения мотора (по умолчанию - 0)
  //Motor1.setLinearSpeed(1);               // по умолчанию скорость 1 мм/с
  //Motor1.setLinearAcceleration(2);        // по умолчанию ускорение 2 мм/с^2

  // перемещение в абсолютной системе координат
  // /*
  Motor1.setCoordinateSystem(0);            // устанавливаем систему координат (по умолчанию - абсолютная)
  Motor1.goLinearPosition(10);              // перемещение в позицию 10 милиметров
  delay(Delay);                             // задержка
  Motor1.goLinearPosition(0);               // перемещение в позицию 0 милиметров
  Serial.println(Motor1.getLinearPosition());  // Получаем конечную позицию в абсолютной системе
  // */

  //перемещение в относительной системе координат
  /*
  Motor1.setCoordinateSystem(1);            // устанавливаем систему координат (по умолчанию - абсолютная)
  Motor1.goLinearPosition(10);              // перемещение на 10 милиметров
  delay(Delay);                             // задержка
  Motor1.goLinearPosition(-10);             // перемещение на -10 милиметров
  Serial.println(Motor1.getLinearPosition());  // Получаем конечную позицию в абсолютной системе
  */
}

void loop() {
  // put your main code here, to run repeatedly:

}
