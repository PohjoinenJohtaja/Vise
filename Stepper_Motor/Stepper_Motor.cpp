#include "Stepper_Motor.h"                                                                  // подключаем заголовок обязательно
#include <Arduino.h>                                                                        // подключаем ардуино

// инициализация системы
Stepper_Motor::Stepper_Motor(uint8_t stepPin, uint8_t dirPin, uint8_t stepDivision) {       // конструктор
    _stepPin = stepPin;                                                                     // запоминаем пин,подключённый к STEP
    _dirPin = dirPin;                                                                       // запоминаем пин,подключённый к DIR
    _stepDivision = stepDivision;                                                           // запоминаем режим микрошага (1, 2, 4...)
    pinMode (_stepPin, OUTPUT);
    pinMode (_dirPin, OUTPUT);
    Stepper_Motor::setDirection(0);
    Stepper_Motor::setGearRatio(1);
    Stepper_Motor::setThreadPitch(1);
    Stepper_Motor::setCoordinateSystem(0);
    Stepper_Motor::setLinearSpeed(1);
    Stepper_Motor::setLinearAcceleration(2);
    Stepper_Motor::setAngularSpeed(1);
    Stepper_Motor::setAngularAcceleration(2);
    }                                                          
void Stepper_Motor::setDirection(bool direction) {_direction = direction;}                  // устанавливаем направление движения мотора
void Stepper_Motor::setGearRatio(float ratio) {_ratio = ratio;}                             // устанавливаем передаточное (ratio > 1 - редуктор; 0 < ratio < 1 - мультипликатор) 
void Stepper_Motor::setThreadPitch(float pitch) {_pitch = pitch;}                           // устанавливаем шаг резьбы (по умолчанию - 2мм)

// методы для взаимодействия с механизмом
void Stepper_Motor::setCoordinateSystem(bool system) {_system = system;}                    // устанавливает систему координат (0 - абсолютаня, 1 - относительная)
void Stepper_Motor::setZeroPosition() {_stepPos = 0;}                                       // установить текущее положение как нулевое

void Stepper_Motor::goLinearPosition(float linPos){                                         // перемещает каретку на координату
    int32_t steps=(int32_t) linPos*_ratio*_stepDivision*200/_pitch - (int32_t) !_system*_stepPos;    // переводим милиметры в шаги
    if (steps>0) moveMotor(steps, 1, 0);                                                    // определяем направление движения
    else moveMotor(-steps, 0, 0);
    _stepPos += steps;                                                                      // запоминаем позицию
}
void Stepper_Motor::setLinearSpeed(float linSpeed) {                                        // устанавливает линейную скорость движения
    _linSpeed = (float) linSpeed*_ratio*_stepDivision*200/_pitch;}
void Stepper_Motor::setLinearAcceleration(float linAccel) {                                 // устанавливает линейное ускорение движения
    _linAccel = (float) linAccel*_ratio*_stepDivision*200/_pitch;}
float Stepper_Motor::getLinearPosition(){                                                   // возвращает абсолютную координату
    return (float)_stepPos*_pitch/_ratio/_stepDivision/200;}
    
void Stepper_Motor::goAngularPosition(float angulPos){                                      // перемещает вал на угол
    int32_t steps=(int32_t) angulPos*_ratio*_stepDivision*200/360 - (int32_t) !_system*_stepPos;// переводим градусы в шаги
    if (steps>0) moveMotor(steps, 1, 1);                                                    // определяем направление движения
    else moveMotor(-steps, 0, 1);
    _stepPos += steps;                                                                      // запоминаем позицию
}
void Stepper_Motor::setAngularSpeed(float anguSpeed){                                       // устанавливает угловую скорость движения
    _anguSpeed = (float) anguSpeed*_ratio*_stepDivision*200;}                               // устанавливает угловую скорость движения
void Stepper_Motor::setAngularAcceleration(float anguAccel){                                // устанавливает угловое ускорение движения
    _anguAccel = (float) anguAccel*_ratio*_stepDivision*200;}
float Stepper_Motor::getAngularPosition(){                                                  // возвращает абсолютный угол
    return (float)_stepPos*1.8/_ratio/_stepDivision;}

void Stepper_Motor::moveMotor(uint32_t steps, bool dir, bool type){                         // метод перемещения (кол-во шагов, направление)
    float Speed;
    float Acceleration;
    if (type) {                                                                             // если type - 1, перемещения угловые
        Speed = _anguSpeed;
        Acceleration = _anguAccel;
    }
    else {                                                                                  // если type - 0, перемещения линейные
        Speed = _linSpeed;
        Acceleration = _linAccel;
    }
    const uint32_t DelayMicros = (uint32_t)500*1000/Speed;                                  // рассчитываем время задержки максимальной скорости
    float delenie = (float) 1000*1000/Acceleration;                                         // переводим ускорение в микросекунды
    uint32_t Vel0 = 10954.45*sqrt(delenie);                                                 // расчитываем первое время для формулы ускорения    
    float delenie2 = (float)Vel0/500/1000/delenie;                                          // отдельная переменная для упрощения расчёта в цикле

    digitalWriteFast(_dirPin, (bool) dir+_direction);                                       // устанавливаем направление

    uint32_t told = micros();                                                               // фиксируем начальное время
    for (uint32_t i=0; i<steps; i++)                                                        // делаем steps шагов
    {
        uint32_t t = micros() - told;                                                       // время, прошедшее с начала выполнения цикла
        uint32_t DelayAccel = (uint32_t) Vel0 / (2 + (float)t*delenie2);                    // рассчитываем время задержки
        if (DelayAccel < DelayMicros) DelayAccel = DelayMicros;                             // если нужная скорость достигнута, a = 0
        
        digitalWriteFast(_stepPin, HIGH);                                                   // подача импульсов
        delayMicroseconds(DelayAccel-1);
        digitalWriteFast(_stepPin, LOW);
        delayMicroseconds(DelayAccel);
  }
}

void Stepper_Motor::digitalWriteFast(uint8_t pin, bool x) {                                 // метод быстрого чтения
  switch (pin) {                                                                            // отключаем ШИМ
    case 3: bitClear(TCCR2A, COM2B1);
      break;
    case 5: bitClear(TCCR0A, COM0B1);
      break;
    case 6: bitClear(TCCR0A, COM0A1);
      break;
    case 9: bitClear(TCCR1A, COM1A1);
      break;
    case 10: bitClear(TCCR1A, COM1B1);
      break;
    case 11: bitClear(TCCR2A, COM2A1);
      break;
  }
  if (pin < 8) {                                                                            //выбираем порт и устанавливаем пин (0, 1)
    bitWrite(PORTD, pin, x);
  } else if (pin < 14) {
    bitWrite(PORTB, (pin - 8), x);
  } else if (pin < 20) {
    bitWrite(PORTC, (pin - 14), x);
  }
}
