#include <Arduino.h>

#include "Balancebot.hpp"
#include "stream_operators.hpp"

Balancebot::Balancebot(BalancebotConfig config)
    : odriveSerial(Serial1),
      odrive(odriveSerial),
      mpu(Wire),
      anglePid(PID(&anglePidInput, &anglePidOutput, &anglePidSetpoint, config.anglePidP, config.anglePidI, config.anglePidD, DIRECT)),
      positionPid(PID(&positionPidInput, &positionPidOutput, &positionPidSetpoint, config.positionPidP, config.positionPidI, config.positionPidD, DIRECT)),
      statusService("19B10000-E8F2-537E-4F6C-D104768A1214"),
      controlService("40760000-8912-48af-9146-b057c465d970"),
      angleCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify),
      targetSpeedCharacteristic("40760001-8912-48af-9146-b057c465d970", BLERead | BLENotify | BLEWriteWithoutResponse),
      targetRotationCharacteristic("40760002-8912-48af-9146-b057c465d970", BLERead | BLENotify | BLEWriteWithoutResponse),
      usePositionHoldCharacteristic("40760003-8912-48af-9146-b057c465d970", BLERead | BLENotify | BLEWriteWithoutResponse),
      isEnabledCharacteristic("40760004-8912-48af-9146-b057c465d970", BLERead | BLENotify | BLEWriteWithoutResponse),
      config(config)
//   odometry(0.0f, 0.0f)
{
    // TODO: configure pid parameters and make configurable from outside
    anglePid.SetMode(AUTOMATIC);
    anglePid.SetOutputLimits(-30, 30);
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
    // setup everything
    setupPins();
    setupMpu();
    setupBluetooth();
    setupOdrive();

    // go to balancing state
    setState(BalancebotState::Balancing);
}

void Balancebot::setupPins()
{
    pinMode(LED_BUILTIN, OUTPUT);
}

void Balancebot::setupMpu()
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
}

void Balancebot::setupBluetooth()
{
    Serial << "Setting up bluetooth low energy..";

    // start bluetooth low energy stack
    if (!BLE.begin())
    {
        Serial << "starting bluetooth low energy module failed!" << '\n';

        return;
    }

    // set name
    BLE.setLocalName("Balancebot");

    // set advertised service
    BLE.setAdvertisedService(statusService);
    // BLE.setAdvertisedService(controlService);

    // add status service characteristics
    statusService.addCharacteristic(angleCharacteristic);

    // add control service characteristics
    controlService.addCharacteristic(targetSpeedCharacteristic);
    controlService.addCharacteristic(targetRotationCharacteristic);
    controlService.addCharacteristic(usePositionHoldCharacteristic);
    controlService.addCharacteristic(isEnabledCharacteristic);

    // add the services
    BLE.addService(statusService);
    BLE.addService(controlService);

    // write initial characteristic values
    reportAllState();

    // start advertising
    BLE.advertise();

    Serial << " done!" << '\n';
}

void Balancebot::setupOdrive()
{
    Serial << "Setting up odrive..";

    // setup odrive serial
    // odriveSerial.begin(115200);

    // using non-standard baudrate!
    // odrv0.config.uart_baudrate = 256000
    odriveSerial.begin(256000);

    // too fast, getting errors
    // odrv0.config.uart_baudrate = 921600
    // odriveSerial.begin(921600);

    // clear odrive errors
    odriveSerial << "sc\n";

    // TODO: configure axes

    Serial << " done!" << '\n';
}

void Balancebot::loop(unsigned long dt, unsigned long currentTimeUs)
{
    loopBluetooth(dt, currentTimeUs, loopIndex);
    loopRobot(dt, currentTimeUs, loopIndex);

    loopIndex++;
}

void Balancebot::loopBluetooth(unsigned long dt, unsigned long currentTimeUs, int loopIndex)
{
    unsigned long startTimeUs = micros();

    bool isBluetoothConnected = BLE.connected();

    // check for change of bluetooth connection state
    if (isBluetoothConnected != wasBluetoothConnected)
    {
        digitalWrite(LED_BUILTIN, isBluetoothConnected);

        wasBluetoothConnected = isBluetoothConnected;

        Serial << "Bluetooth " << (isBluetoothConnected ? "connected" : "disconnected") << '\n';

        // stop the robot if bluetooth connection is lost
        if (!isBluetoothConnected)
        {
            targetSpeed = 0;
            targetRotation = 0;
            targetPosition = 0.0f;
            usePositionHold = true;

            reportAllState();

            Serial << "Bluetooth connection was lost, stopping robot is position hold mode\n";
        }
    }

    // nothing to do if not connected
    if (!isBluetoothConnected)
    {
        return;
    }

    // increment target distance in position hold mode
    if (usePositionHold)
    {
        targetPosition += (float)targetSpeed * Balancebot::SPEED_DISTANCE_MULTIPLIER * ((float)dt / 1000000.0f);
    }
    else
    {
        targetPosition = 0.0f;
    }

    // poll bluetooth for updates
    // BLE.poll();

    // attempt to get central device
    BLEDevice central = BLE.central();

    if (central)
    {
        // Serial << "Connected to central: " << central.address() << '\n';

        // handle target speed updates
        if (targetSpeedCharacteristic.written())
        {
            targetSpeed = targetSpeedCharacteristic.value();

            targetSpeedCharacteristic.writeValue(targetSpeed);

            Serial << "Got new target speed of " << targetSpeed << '\n';

            // make the robot stop at the place zero speed was requested
            if (targetSpeed == 0 && usePositionHold)
            {
                targetPosition = lastPosition - positionHoldStartPosition;

                Serial << "Stopping requested, updating target position to " << targetPosition << '\n';
            }
        }

        // handle target rotation updates
        if (targetRotationCharacteristic.written())
        {
            targetRotation = targetRotationCharacteristic.value();

            targetRotationCharacteristic.writeValue(targetRotation);

            Serial << "Got new target rotation of " << targetRotation << '\n';
        }

        // handle position hold updates
        if (usePositionHoldCharacteristic.written())
        {
            usePositionHold = usePositionHoldCharacteristic.value();

            usePositionHoldCharacteristic.writeValue(usePositionHold);

            Serial << "Position hold is now " << (usePositionHold ? "enabled" : "disabled") << '\n';

            // update the initial position hold start position once enabled
            if (usePositionHold)
            {
                updatePositionHoldStartPosition();
            }

            // reset target distance
            targetPosition = 0.0f;
        }

        // handle is enabled updates
        if (isEnabledCharacteristic.written())
        {
            isEnabled = isEnabledCharacteristic.value();

            isEnabledCharacteristic.writeValue(isEnabled);

            Serial << "Robot is now " << (isEnabled ? "enabled" : "disabled") << '\n';

            // reset target distance
            targetPosition = 0.0f;

            if (isEnabled && usePositionHold)
            {
                updatePositionHoldStartPosition();
            }
        }

        // calculate time since last reported angle
        unsigned long timeSinceReportedAngleUs = currentTimeUs - lastAngleReportedTimeUs;

        // report robot angle at certain interval
        if (timeSinceReportedAngleUs > REPORT_ANGLE_INTERVAL_US)
        {
            angleCharacteristic.writeValue(angle);

            lastAngleReportedTimeUs = currentTimeUs;
        }
    }

    unsigned long timeTakenUs = micros() - startTimeUs;

    // log abnormally slow updates
    if (timeTakenUs > 10000)
    {
        Serial << "Slow update of bluetooth took " << timeTakenUs << "us\n";
    }
}

void Balancebot::loopRobot(unsigned long dt, unsigned long currentTimeUs, int loopIndex)
{
    unsigned long startTimeUs = micros();

    // handle states
    switch (state)
    {
    case BalancebotState::Balancing:
        loopRobotBalacing(dt, currentTimeUs, loopIndex);
        break;

    default:
        // ignore other states
        break;
    }

    unsigned long timeTakenUs = micros() - startTimeUs;

    // log abnormally slow updates
    if (timeTakenUs > 30000)
    {
        Serial << "Slow update of robot took " << timeTakenUs << "us\n";
    }
}

void Balancebot::loopRobotBalacing(unsigned long dt, unsigned long currentTimeUs, int loopIndex)
{
    // TODO: getting odometry is slow.. calculate from requested velocities?
    // only fetch odometry data if position hold is enabled
    BalancebotOdometry odometry = usePositionHold ? getOdometry() : BalancebotOdometry(0.0f, 0.0f);

    // store last position
    lastPosition = odometry.position;

    // calculate position holding PID controller
    positionPidInput = odometry.position;
    positionPidSetpoint = positionHoldStartPosition + targetPosition;
    positionPid.Compute();

    // update the inertial measurement unit
    mpu.update();

    // get robot angle and calculate limited velocity
    angle = mpu.getAngleY();

    // calculate target angles based on speed or position hold
    double speedTargetAngle = (double)targetSpeed;
    double positionPidTargetAngle = positionPidOutput;

    // calculate angle holding PID controller
    anglePidInput = angle;
    anglePidSetpoint = usePositionHold ? idleTargetAngle + positionPidTargetAngle : idleTargetAngle + speedTargetAngle;
    anglePid.Compute();

    // calculate motor velocities applying turning rotation
    double targetVelocity = anglePidOutput;
    double rotationVelocity = targetRotation * Balancebot::ROTATION_VELOCITY_MULTIPLIER;
    double targetLeftVelocity = targetVelocity + rotationVelocity;
    double targetRightVelocity = targetVelocity - rotationVelocity;

    // consider robot having fallen over if the angle is too large
    bool hasRobotFallenOver = abs(angle) > 30.0f;
    bool wasRobotFallenOverRecently = currentTimeUs - lastRobotFallenOverTimeUs < Balancebot::ROBOT_FALLEN_OVER_RECENTLY_TIME_US;

    // update last robot fallen over time
    if (hasRobotFallenOver)
    {
        lastRobotFallenOverTimeUs = currentTimeUs;
    }

    // stop motors if robot has fallen over (recently) or is disabled
    if (hasRobotFallenOver || wasRobotFallenOverRecently || !isEnabled)
    {
        targetLeftVelocity = 0.0f;
        targetRightVelocity = 0.0f;
    }

    // reset target position and position hold start position if robot just fell over
    if (hasRobotFallenOver && !wasRobotFallenOver)
    {
        targetPosition = 0.0f;
        updatePositionHoldStartPosition();

        Serial << "Robot has fallen over\n";
    }
    else if (!hasRobotFallenOver && wasRobotFallenOver)
    {
        Serial << "Robot got back up\n";
    }

    setMotorVelocities(targetLeftVelocity, targetRightVelocity);

    // update fallen over state
    wasRobotFallenOver = hasRobotFallenOver;

    // TODO: this is not really accurate enough
    // update the fake odometry based on last requested velocities
    // odometry.leftPosition += motorVelocityLeft * ((float)dt / (float)Balancebot::US_IN_SECOND);
    // odometry.rightPosition += motorVelocityRight * ((float)dt / (float)Balancebot::US_IN_SECOND);
    // odometry.position = (odometry.leftPosition + odometry.rightPosition) / 2.0f;

    // log for serial plotter
    // Serial << (currentTimeUs / 1000.0f) << '\t' << angle << '\t' << anglePidSetpoint << '\t' << targetVelocity << '\t' << odometry.leftPosition << '\t' << odometry.rightPosition << '\t' << odometry.position << '\t' << dt << '\n';

    // log at interval
    // if (loopIndex % 100 == 0)
    // {
    //     Serial << "Odometry: " << odometry.leftPosition << ", " << odometry.rightPosition << " - " << odometry.position << ", dt: " << dt << '\n';
    //     // Serial << "Setpoint: " << anglePidSetpoint << ", real: " << angle << ", dt: " << dt << '\n';
    //     // Serial << "Angle setpoint: " << anglePidSetpoint << ", target distance: " << positionPidSetpoint << ", real distance: " << odometry.position << '\n';
    // }
}

void Balancebot::setState(BalancebotState newState)
{
    if (newState == state)
    {
        Serial << "Requested to transition to state " << getStateName(newState) << " but the robot is already in given state" << '\n';

        return;
    }

    BalancebotState oldState = state;
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

    Serial << "Transitioned from " << getStateName(oldState) << " to " << getStateName(newState) << "state\n";
}

void Balancebot::setError(String newError)
{
    error = newError;

    setState(BalancebotState::Error);
}

void Balancebot::reportAllState()
{
    angleCharacteristic.writeValue(angle);
    targetSpeedCharacteristic.writeValue(targetSpeed);
    targetRotationCharacteristic.writeValue(targetRotation);
    usePositionHoldCharacteristic.writeValue(usePositionHold);
    isEnabledCharacteristic.writeValue(isEnabled);
}

void Balancebot::updatePositionHoldStartPosition()
{
    BalancebotOdometry odometry = getOdometry();

    positionHoldStartPosition = odometry.position;

    // TODO: how to reset pid controller, do we need to?
    // positionPid.Reset();

    Serial << "Updated position hold start position to: " << positionHoldStartPosition << '\n';
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

    // update current requested motor velocities
    motorVelocityLeft = velocityLeft;
    motorVelocityRight = velocityRight;
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
    // TODO: any way to make this faster?
    // get motor position estimates
    // odriveSerial << "r axis0.encoder.pos_estimate\n";
    // float rightPosition = odrive.readFloat() * -1.0f;
    // odriveSerial << "r axis1.encoder.pos_estimate\n";
    // float leftPosition = odrive.readFloat();

    // get motor position estimates
    odriveSerial << "f 0\n";
    float rightPosition = odrive.readFloat() * -1.0f;
    odriveSerial << "f 1\n";
    float leftPosition = odrive.readFloat();

    // Serial << "feedback: " << r << '\n';
    // TODO: extract position and velocity from string "0.0 0.0" etc

    // detect abnormal odometry readings (big difference from last reading)
    // if (abs(odometry.position) > 0.1f && (abs(odometry.leftPosition - leftPosition) > 1000 || abs(odometry.rightPosition - rightPosition) > 1000))
    // {
    //     Serial << "Got abnormal odometry: " << leftPosition << ", " << rightPosition << " (previously " << odometry.leftPosition << ", " << odometry.rightPosition << ")" << '\n';
    // }

    // TODO: calculate real distances using wheel diameter
    return BalancebotOdometry(leftPosition, rightPosition);
}