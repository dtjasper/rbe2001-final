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
    long getPositionB();
    void resetA();
    void resetB();
    void setup();

private:
    void setEffortMotorA(int effort, bool clockwise);
    void setEffortMotorB(int effort, bool clockwise);
    static void isrA();
    static void isrB();
    const int tolerance = 3;
    const int PWMAOutPin = 5;
    const int AIN2 = 12;
    const int AIN1 = 11;

    const int PWMBOutPin = 3;
    const int BIN2 = 13;
    const int BIN1 = 6;
};