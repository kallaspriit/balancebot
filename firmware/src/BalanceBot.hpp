#ifndef BALANCEBOT_HPP
#define BALANCEBOT_HPP

#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include <MPU6050_light.h>
#include <Wire.h>

enum BalancebotState
{
    Initializing = 0,
    Error = 1,
    Balancing = 2,
};

class BalanceBot
{
public:
    BalanceBot();
    ~BalanceBot();

    void setup();
    void loop(unsigned long dt, unsigned long currentTime);

private:
    String getStateName(BalancebotState state);

    void setMotorVelocities(float velocityLeft, float velocityRight);

    void setState(BalancebotState newState);
    void setError(String error);

    void onEnterBalacingState();

    void loopBalacingState(unsigned long dt, unsigned long currentTime);

    // dependencies
    HardwareSerial &odriveSerial;
    ODriveArduino odrive;
    MPU6050 mpu;

    // runtime state
    BalancebotState state = BalancebotState::Initializing;
    String error = "";
};

#endif // BALANCEBOT_HPP