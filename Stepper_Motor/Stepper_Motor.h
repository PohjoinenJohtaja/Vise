#ifndef Stepper_Motor_h
#define Stepper_Motor_h
// описание класса
class Stepper_Motor {   // класс Stepper_Motor
public:
    Stepper_Motor(byte PIN_STEP, byte PIN_DIR, byte step_division);
    void setColor(byte color);
    void setBright(byte bright);
    byte getColor();
    byte getBright();
private:
    byte _color;  // переменная цвета
    byte _bright; // переменная яркости
};
#endif
