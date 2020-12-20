#ifndef Stepper_Motor_h
#define Stepper_Motor_h
// описание класса
class Stepper_Motor {                                                   // класс Stepper_Motor
public:                                                                 // открытые методы
    // методы для инициализации механизма
    Stepper_Motor(uint8_t stepPin, uint8_t dirPin, uint8_t stepDivision = 1); // инициализация мотора 
    void setDirection(bool direction = 0);                              // установка направления вращения
    void setGearRatio(float ratio = 1);                                 // задание передаточного отношения
    void setThreadPitch(float pitch = 1);                               // задание шага резьбы виинта
    void setRailLength(uint16_t lenght);                                // задание длины направляющей

    // методы для взаимодействия с механизмом
    void setCoordinateSystem(bool system = 0);                          // устанавливает систему координат (абсолютаня, относительная)
    void setZeroPosition();                                             // установить текущее положение как нулевое

    void goLinearPosition(float linPos);       // перемещает каретку на координату
    void setLinearSpeed(uint16_t linSpeed = 75);                       // устанавливает линейную скорость движения
    void setLinearAcceleration(uint16_t linAccel = 15);                 // устанавливает линейное ускорение движения
    float getLinearPosition();                                          // возвращает абсолютную координату
    
    void goAngularPosition(float angulPos);    // перемещает вал на угол
    void setAngularSpeed(uint16_t anguSpeed = 75);                      // устанавливает угловую скорость движения
    void setAngularAcceleration(uint16_t anguAccel = 15);               // устанавливает угловое ускорение движения
    float getAngularPosition();                                         // возвращает абсолютный угол
    
private:                                                                // закрытые методы
    uint8_t _stepPin;                                                   // пин, подключаемый к STEP драйвера
    uint8_t _dirPin;                                                    // пин, подключаемый к DIR драйвера
    uint8_t _stepDivision;                                              // режим шага двигателя (1, 2, 4, 8, 16)
    bool _direction;                                                    // константа направления вращения
    float _ratio;                                                       // константа передаточного отношения
    float _pitch;                                                       // константа шага резьбы виинта
    uint16_t _lenght;                                                   // константа длины направляющей (мм)

    bool _system;                                                       // система координат (0 = абсолютная; 1 = относительная)
    uint16_t _linSpeed;                                                 // установка линейной скорости (мм/мин)
    uint16_t _linAccel;                                                 // установка линейного ускорения (мм/мин^2)
    uint16_t _anguSpeed;                                                // установка угловой скорости (об/мин)
    uint16_t _anguAccel;                                                // установка углового ускорения (об/мин^2)
    uint32_t _stepPos = 0;                                              // запоминает положение вала

    void moveMotor(uint32_t steps, bool dir, bool type);                // метод перемещения (кол-во шагов, направление)
};
#endif
