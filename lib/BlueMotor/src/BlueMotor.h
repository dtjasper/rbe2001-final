#pragma once

class BlueMotor
{
public:
    BlueMotor();
    void setEffortA(int effort);
    void setEffortB(int effort);
    void moveToMotorA(long position);
    long getPositionA();
    void moveToMotorB(long position);
    int getMotorAEffort();
    int getMotorBEffort();
    long getPositionB();
    void goodSetEffort(int effort, bool motor); // TRUE is A and FALSE is B
    void resetA();
    void resetB();
    void setup();

private:
    void setEffortMotorA(int effort, bool clockwise);
    void setEffortMotorB(int effort, bool clockwise);
    void goodSetEffortMotor(int effort, bool clockwise, bool aOrB);
    static void isrA();
    static void isrB();
    const int tolerance = 3;
    const int PWM_MAIN = 3;
    const int PWMAOutPin = 5;
    const int AIN2 = 22;
    const int AIN1 = 11;

    const int PWMBOutPin = 3;
    const int BIN2 = 13;
    const int BIN1 = 6;
};