#include <Arduino.h>
#include "BlueMotor.h"


// MOTOR A COUNTERS/TIMERS
long oldValueA = 0;
long newValueA;
volatile long countA = 0;
unsigned timeA = 0;
// MOTOR B COUNTERS/TIMERS
long oldValueB = 0;
long newValueB;
volatile long countB = 0;
unsigned timeB = 0;

// Motor A Encoders 1&2
const int MotorAENCA = 4;
const int MotorAENCB = 2;
// Motor B Encoders 1&2
const int MotorBENCA = 0;
const int MotorBENCB = 1;

// VALUES FOR ENCODER TRACKING MOTORS A
int MotorA_oldEncoderA = 0;
int MotorA_oldEncoderB = 0;
int MotorA_newEncoderA = 0;
int MotorA_newEncoderB = 0;
// VALUES FOR ENCODER TRACKING MOTORS B
int MotorB_oldEncoderA = 0;
int MotorB_oldEncoderB = 0;
int MotorB_newEncoderA = 0;
int MotorB_newEncoderB = 0;

//Motor Effort values for getters
int MotorAEffort = 0;
int MotorBEffort = 0;

// COMPILER
BlueMotor::BlueMotor()
{
}

// SETUP (should work no matter A or B)
void BlueMotor::setup()
{
    pinMode(PWMAOutPin, OUTPUT);
    pinMode(PWMBOutPin, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(MotorAENCA, INPUT);
    pinMode(MotorAENCB, INPUT);
    pinMode(MotorBENCA, INPUT);
    pinMode(MotorBENCB, INPUT);
    /*
    TCCR1A = 0xA8; //0b10101000; //gcl: added OCR1C for adding a third PWM on pin 11
    TCCR1B = 0x11; //0b00010001;
    ICR1 = 400;
    OCR1C = 0;*/

    //TCCR3A = 0;
    //TCCR0B = 0; 
   // TCCR3A = (1 << COM3A1) | (1 << WGM30);
    //TCCR3B = (1 << WGM32) | (1 << CS31) | (1 << CS30); // prescaler 64

    OCR3A = 0;
    //OCR0B = 0;

    // ICR1 = 400; // for precise timing with encoders on motor a

    // attach an interrupt for both encoders in A
    attachInterrupt(digitalPinToInterrupt(MotorAENCA), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MotorAENCB), isrA, CHANGE);
    // attach an interrupt for both encoders in B
    attachInterrupt(digitalPinToInterrupt(MotorBENCA), isrB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MotorBENCB), isrB, CHANGE);
    // RESET ENCODERS FOR BOTH MOTORS
    resetA();
    resetB();
}

// GETS ENCODER POSITION OF MOTOR A 
long BlueMotor::getPositionA()
{
    long tempCountA = 0;
    noInterrupts();
    tempCountA = countA;
    interrupts();
    return tempCountA;
}

// GETS ENCODER POSITION OF MOTOR B
long BlueMotor::getPositionB()
{
    long tempCountB = 0;
    noInterrupts();
    tempCountB = countB;
    interrupts();
    return tempCountB;
}

// RESETS ENCODER POSITION OF MOTOR A
void BlueMotor::resetA()
{
    noInterrupts();
    countA = 0;
    interrupts();
}

// RESETS ENCODER POSITION OF MOTOR B
void BlueMotor::resetB()
{
    noInterrupts();
    countB = 0;
    interrupts();
}

// INTERUPT FOR MOTOR A ENCODERS
void BlueMotor::isrA()
{
    MotorA_newEncoderA = digitalRead(MotorAENCA);
    MotorA_newEncoderB = digitalRead(MotorAENCB);

    if(((MotorA_oldEncoderB == LOW && MotorA_newEncoderB == LOW ) && (MotorA_oldEncoderA == LOW && MotorA_newEncoderA == HIGH)) ||
        ((MotorA_oldEncoderA == HIGH && MotorA_newEncoderA == LOW) && (MotorA_oldEncoderB == HIGH && MotorA_newEncoderB == HIGH )) ||
        ((MotorA_oldEncoderA == HIGH && MotorA_newEncoderA == HIGH ) && (MotorA_oldEncoderB == LOW && MotorA_newEncoderB == HIGH)) ||
        ((MotorA_oldEncoderA == LOW && MotorA_newEncoderA == LOW) && (MotorA_oldEncoderB == HIGH && MotorA_newEncoderB == LOW )))
        {
            countA++;
        }
    else
        {
            countA--;
        }

    MotorA_oldEncoderA = MotorA_newEncoderA;
    MotorA_oldEncoderB = MotorA_newEncoderB;

}

// INTERUPT FOR MOTOR B ENCODERS
void BlueMotor::isrB()
{
    MotorB_newEncoderA = digitalRead(MotorBENCA);
    MotorB_newEncoderB = digitalRead(MotorBENCB);

    if(((MotorB_oldEncoderB == LOW && MotorB_newEncoderB == LOW ) && (MotorB_oldEncoderA == LOW && MotorB_newEncoderA == HIGH)) ||
        ((MotorB_oldEncoderA == HIGH && MotorB_newEncoderA == LOW) && (MotorB_oldEncoderB == HIGH && MotorB_newEncoderB == HIGH )) ||
        ((MotorB_oldEncoderA == HIGH && MotorB_newEncoderA == HIGH ) && (MotorB_oldEncoderB == LOW && MotorB_newEncoderB == HIGH)) ||
        ((MotorB_oldEncoderA == LOW && MotorB_newEncoderA == LOW) && (MotorB_oldEncoderB == HIGH && MotorB_newEncoderB == LOW )))
        {
            countB++;
        }
    else
        {
            countB--;
        }

    MotorB_oldEncoderA = MotorB_newEncoderA;
    MotorB_oldEncoderB = MotorB_newEncoderB;

}

void BlueMotor::goodSetEffort(int effort, bool motor){
    if (effort < 0)
    {
        goodSetEffortMotor(-effort, true, motor);
    }
    else 
    {
        goodSetEffortMotor(effort, false, motor);
    }
    
}

void BlueMotor::goodSetEffortMotor(int effort, bool clockwise, bool aOrB){
    if(aOrB && effort == 0){
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
    }
    else if(!aOrB && effort == 0){
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
    }
    else if(aOrB && effort !=0){
        if (clockwise)
        {
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
        }
        else
        {
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, HIGH);
        }
        analogWrite(PWM_MAIN, (effort, 0, 400)); // unsure
    }
    else{
        if (clockwise)
        {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        }
        else
        {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        }
        analogWrite(PWM_MAIN, (effort, 0, 400)); // unsure
    }
}

// SETS EFFORT FOR MOTOR A (public)
void BlueMotor::setEffortA(int effort)
{
    Serial.print("Setting Effort on Motor A");
    if (effort < 0)
    {
        setEffortMotorA(-effort, true);
    }
    else
    {
        setEffortMotorA(effort, false);
    }
}

// SETS EFFORT FOR MOTOR B (public)
void BlueMotor::setEffortB(int effort)
{   Serial.print("Setting Effort on Motor B");
    if (effort < 0)
    {
        setEffortMotorB(-effort, true);
    }
    else
    {
        setEffortMotorB(effort, false);
    }
}

// SETS EFFORT FOR MOTOR A (private, with direction)
void BlueMotor::setEffortMotorA(int effort, bool clockwise)
{
    MotorAEffort = effort;
    if (clockwise)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    OCR3A = constrain(effort, 0, 400); // unsure
}

// SETS EFFORT FOR MOTOR B (private, with direction)
void BlueMotor::setEffortMotorB(int effort, bool clockwise)
{
    MotorBEffort = effort;
    if (clockwise)
    {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    }
    else
    {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
    }
    analogWrite(PWMBOutPin, (effort, 0, 400)); // unsure
}

// MOVES MOTOR A TO A POSITION (target)
void BlueMotor::moveToMotorA(long target)  //Move to this encoder position within the specified
{                                    //tolerance in the header file using proportional control
                                     //then stop
    
    setEffortA(0);
}

// MOVES MOTOR B TO A POSITION (target)
void BlueMotor::moveToMotorB(long target)  //Move to this encoder position within the specified
{                                    //tolerance in the header file using proportional control
                                     //then stop
    
    setEffortB(0);
}
int BlueMotor::getMotorAEffort(){
    return MotorAEffort;
}
int BlueMotor::getMotorBEffort(){
    
}

