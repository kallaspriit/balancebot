#include <Arduino.h>

#include "Balancebot.hpp"
#include "stream_operators.hpp"

Balancebot::Balancebot(BalancebotConfig config)
    : odriveSerial(Serial1),
      odrive(odriveSerial),
      mpu(Wire),
      anglePid(PID(&anglePidInput, &anglePidOutput, &anglePidSetpoint, config.anglePidP, config.anglePidI, config.anglePidD, DIRECT)),
      positionPid(PID(&positionPidInput, &positionPidOutput, &positionPidSetpoint, config.positionPidP, config.positionPidI, config.positionPidD, DIRECT)),
      config(config)
{
    // TODO: configure pid parameters and make configurable from outside
    anglePid.SetMode(AUTOMATIC);
    anglePid.SetOutputLimits(-20, 20);
    anglePid.SetSampleTime(10);

    positionPid.SetMode(AUTOMATIC);
    // positionPid.SetOutputLimits(-8, 8);
    positionPid.SetOutputLimits(-4, 4);
    positionPid.SetSampleTime(10);
}

Balancebot::~Balancebot()
{
}

void Balancebot::setup()
{
    // setup I2C bus
    Wire.begin();

    Serial << "Searching for MPU6050..";

    // attempt to open I2C connection to the IMU
    if (mpu.begin() != 0)
    {
        Serial << " failed!" << '\n';

        setError("Setting up MPU6050 IMU failed");

        return;
    }

    Serial << " done!" << '\n';

    Serial << "Calculating IMU offsets, keep still.. ";

    // calculate IMU offsets
    delay(1000);
    mpu.calcOffsets();

    Serial << " done!" << '\n';

    Serial << "Setting up odrive..";

    // setup odrive serial
    odriveSerial.begin(115200);

    Serial << " done!" << '\n';

    // go to balancing state
    setState(BalancebotState::Balancing);
}

void Balancebot::loop(unsigned long dt, unsigned long currentTime)
{
    // handle states
    switch (state)
    {
    case BalancebotState::Balancing:
        loopBalacingState(dt, currentTime);
        break;

    default:
        // ignore
        break;
    }
}

void Balancebot::loopBalacingState(unsigned long dt, unsigned long currentTime)
{
    // TODO: get from configuration / potentiometer
    double idleTargetAngle = 0.0;

    // TODO: get target distance from remote controller
    double targetDistance = 0.0;

    // get robot position and averageDistance travelled from encoders
    BalancebotOdometry odometry = getOdometry();

    // calculate position holding PID controller
    positionPidInput = -odometry.averageDistance;
    positionPidSetpoint = targetDistance;
    positionPid.Compute();
    double positionPidTargetAngle = positionPidOutput;

    // don't use position hold at the beginning
    // would require storing target offset at first position hold
    // if (currentTime < 30) {
    //     positionPidTargetAngle = 0.0;
    // }

    // update the inertial measurement unit
    mpu.update();

    // get robot angle and calculate limited velocity
    float angle = mpu.getAngleY();

    // calculate angle holding PID controller
    anglePidInput = angle;
    anglePidSetpoint = idleTargetAngle + positionPidTargetAngle;
    anglePid.Compute();
    double targetVelocity = anglePidOutput;

    // stop motors if angle gets too big
    if (abs(angle) > 20.0f)
    {
        targetVelocity = 0.0f;
    }

    // TODO: implement steering

    // set motor velocities
    setMotorVelocities(targetVelocity, targetVelocity);

    // log for serial plotter
    // Serial << (currentTime / 1000.0f) << '\t' << angle << '\t' << anglePidSetpoint << '\t' << targetVelocity << '\t' << odometry.leftDistance << '\t' << odometry.rightDistance << '\t' << odometry.averageDistance << '\t' << dt << '\n';
}

void Balancebot::setState(BalancebotState newState)
{
    if (newState == state)
    {
        Serial << "Requested to transition to state " << getStateName(newState) << " but the robot is already in given state" << '\n';
    }

    state = newState;

    // handle entering states
    switch (state)
    {
    case BalancebotState::Balancing:
        onEnterBalacingState();

    default:
        // ignore
        break;
    }

    Serial << "Transitioned from state " << getStateName(state) << " to " << getStateName(newState) << '\n';
}

void Balancebot::setError(String newError)
{
    error = newError;

    setState(BalancebotState::Error);
}

void Balancebot::onEnterBalacingState()
{
    // use velocity control with velocity ramp
    odriveSerial << "w axis0.controller.config.control_mode " << CONTROL_MODE_VELOCITY_CONTROL << '\n';
    odriveSerial << "w axis0.controller.config.input_mode " << INPUT_MODE_VEL_RAMP << '\n';
}

void Balancebot::setMotorVelocities(float velocityLeft, float velocityRight)
{
    // M0 is right, M1 is left and inverted
    odrive.SetVelocity(0, velocityRight);
    odrive.SetVelocity(1, -velocityLeft);
}

String Balancebot::getStateName(BalancebotState state)
{
    switch (state)
    {
    case BalancebotState::Initializing:
        return "Initializing";

    case BalancebotState::Error:
        return "Error";

    case BalancebotState::Balancing:
        return "Balancing";

    default:
        return "Unknown";
    }
}

BalancebotOdometry Balancebot::getOdometry()
{
    // get motor position estimates
    odriveSerial << "r axis" << 0 << ".encoder.pos_estimate << \n";
    float rightDistance = odrive.readFloat();
    odriveSerial << "r axis" << 1 << ".encoder.pos_estimate << \n";
    float leftDistance = odrive.readFloat() * -1;

    // TODO: calculate real distances using wheel diameter
    return BalancebotOdometry(leftDistance, rightDistance);
}