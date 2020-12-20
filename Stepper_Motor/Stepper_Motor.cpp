#include "Stepper_Motor.h"                                                                  // подключаем заголовок обязательно
#include <Arduino.h>                                                                        // подключаем ардуино

// инициализация системы
Stepper_Motor::Stepper_Motor(uint8_t stepPin, uint8_t dirPin, uint8_t stepDivision = 1) {   // конструктор
    _stepPin = stepPin;                                                                     // запоминаем пин,подключённый к STEP
    _dirPin = dirPin;                                                                       // запоминаем пин,подключённый к DIR
    _stepDivision = stepDivision;}                                                          // запоминаем режим микрошага (1, 2, 4...)
void Stepper_Motor::setDirection(bool direction = 0) {_direction = direction;}              // устанавливаем направление движения мотора
void Stepper_Motor::setGearRatio(float ratio = 1) {_ratio = ratio;}                         // устанавливаем передаточное (ratio > 1 - редуктор; 0 < ratio < 1 - мультипликатор) 
void Stepper_Motor::setThreadPitch(float pitch = 2) {_pitch = pitch;}                       // устанавливаем шаг резьбы (по умолчанию - 2мм)
void Stepper_Motor::setRailLength(uint16_t lenght) {_lenght = lenght;}                      // устанавливаем рабочую длину направляющей (в мм)

// методы для взаимодействия с механизмом
void Stepper_Motor::setCoordinateSystem(bool system = 0) {_system = system;}                // устанавливает систему координат (0 - абсолютаня, 1 - относительная)
void Stepper_Motor::setZeroPosition() {_stepPos = 0;}                                       // установить текущее положение как нулевое

void Stepper_Motor::goLinearPosition(float linPos){                                         // перемещает каретку на координату
    float dx = linPos - !_system * (float)_stepPos*_pitch/_ratio/_division/200;             // считаем на сколько надо переместиться
    long steps=(long) dx*_ratio*_stepDivision*200/_pitch;                                   // переводим милиметры в шаги
    if (dx>0) moveMotor(steps, 1);                                                          // определяем направление движения
    else moveMotor(-steps, 0);
    _stepPos += steps;                                                                      // запоминаем позицию
}
void Stepper_Motor::setLinearSpeed(uint16_t linSpeed = 75) {_linSpeed = linSpeed;}          // устанавливает линейную скорость движения
void Stepper_Motor::setLinearAcceleration(uint16_t linAccel = 15) {_linAccel = linAccel;}   // устанавливает линейное ускорение движения
float Stepper_Motor::getLinearPosition(){                                                   // возвращает абсолютную координату
    return (float)_stepPos*_pitch/_ratio/_stepDivision/200;}
    
void Stepper_Motor::goAngularPosition(float angulPos){                                      // перемещает вал на угол
    float dalfa = angulPos - !_system * (float)_stepPos*_pitch*1.8/_ratio/_division;        // считаем на сколько градусов надо переместиться
    long steps=(long) dalfa*_ratio*_stepDivision/_pitch/1.8;                                // переводим градусы в шаги
    if (dalfa>0) moveMotor(steps, 1);                                                          // определяем направление движения
    else moveMotor(-steps, 0);
    _stepPos += steps;                                                                      // запоминаем позицию
}
void Stepper_Motor::setAngularSpeed(uint16_t anguSpeed = 75){_anguSpeed = anguSpeed;}       // устанавливает угловую скорость движения
void Stepper_Motor::setAngularAcceleration(uint16_t anguAccel = 15){_anguAccel = anguAccel;}// устанавливает угловое ускорение движения
float Stepper_Motor::getAngularPosition(){                                                  // возвращает абсолютный угол
    return (float)_stepPos*_pitch*1.8/_ratio/_stepDivision;}

void Stepper_Motor::moveMotor(uint32_t steps, bool dir, bool type){                         // метод перемещения (кол-во шагов, направление)
    if (type) {                                                                             // если type - 1, перемещения угловые
        uint16_t speed = _anguSpeed;
        uint16_t acceleration = _anguAccel;
    }
    else {                                                                                  // если type - 0, перемещения линейные
        uint16_t speed = _linSpeed;
        uint16_t acceleration = _linAccel;
    }
    const long DelayMicros = (long)30*1000*1000/speed/_stepDivision;                        // рассчитываем время задержки
    long told = micros();
    for (long i=0; i<steps; i++)
    {
        float accel = (float)acceleration/30*1000*1000;
        float Sqrt = sqrt((long)2*30*1000*1000/acceleration);
        long t = micros() - told;
        long DelayAccel = (long) Sqrt / (1 + (long)t*Sqrt*acceleration/2/30/1000/1000);

        if (DelayAccel < DelayMicros) DelayAccel = DelayMicros;
    
        digitalWrite(_dirPin, (bool) dir+_direction);
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(DelayAccel-5);
        digitalWrite(_stepPin, LOW);
        delayMicroseconds(DelayAccel-10);
  }
}
