#ifndef Stepper_Motor_h
#define Stepper_Motor_h
#include <Arduino.h>
// описание класса
class Stepper_Motor {                                                       // класс Stepper_Motor
public:                                                                     // открытые методы
    // методы для инициализации механизма
    Stepper_Motor(uint8_t stepPin, uint8_t dirPin, uint8_t stepDivision);   // инициализация мотора 
    void setDirection(bool direction);                                      // установка направления вращения
    void setGearRatio(float ratio);                                         // задание передаточного отношения
    void setThreadPitch(float pitch);                                       // задание шага резьбы виинта

    // методы для взаимодействия с механизмом
    void setCoordinateSystem(bool system);                                  // устанавливает систему координат (абсолютаня, относительная)
    void setZeroPosition();                                                 // установить текущее положение как нулевое

    void goLinearPosition(float linPos);                                    // перемещает каретку на координату
    void setLinearSpeed(float linSpeed);                                    // устанавливает линейную скорость движения
    void setLinearAcceleration(float linAccel);                             // устанавливает линейное ускорение движения
    float getLinearPosition();                                              // возвращает абсолютную координату
    
    void goAngularPosition(float angulPos);                                 // перемещает вал на угол
    void setAngularSpeed(float anguSpeed);                                  // устанавливает угловую скорость движения
    void setAngularAcceleration(float anguAccel);                           // устанавливает угловое ускорение движения
    float getAngularPosition();                                             // возвращает абсолютный угол
    
private:                                                                    // закрытые методы
    uint8_t _stepPin;                                                       // пин, подключаемый к STEP драйвера
    uint8_t _dirPin;                                                        // пин, подключаемый к DIR драйвера
    uint8_t _stepDivision;                                                  // режим шага двигателя (1, 2, 4, 8, 16, 32)
    bool _direction;                                                        // константа направления вращения
    float _ratio;                                                           // константа передаточного отношения
    float _pitch;                                                           // константа шага резьбы винта
    uint16_t _lenght;                                                       // константа длины направляющей (мм)

    bool _system;                                                           // система координат (0 = абсолютная; 1 = относительная)
    float _linSpeed;                                                        // установка линейной скорости (мм/мин)
    float _linAccel;                                                        // установка линейного ускорения (мм/мин^2)
    float _anguSpeed;                                                       // установка угловой скорости (об/мин)
    float _anguAccel;                                                       // установка углового ускорения (об/мин^2)
    int64_t _stepPos = 0;                                                   // запоминает положение вала
    
    void moveMotor(uint32_t steps, bool dir, bool type);                    // метод перемещения (кол-во шагов, направление)
    void digitalWriteFast(uint8_t pin, bool x);                             // метод быстрого чтения состояния пина
};
#endif
