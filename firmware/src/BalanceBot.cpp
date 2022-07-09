#include <Arduino.h>

#include "Balancebot.hpp"
#include "stream_operators.hpp"

Balancebot::Balancebot(BalancebotConfig config)
    : config(config),
      odriveSerial(Serial1),
      odrive(odriveSerial),
      mpu(Wire),
      anglePid(PID(&anglePidInput, &anglePidOutput, &anglePidSetpoint, config.anglePidP, config.anglePidI, config.anglePidD, DIRECT)),
      positionPid(PID(&positionPidInput, &positionPidOutput, &positionPidSetpoint, config.positionPidP, config.positionPidI, config.positionPidD, DIRECT)),
      rxSerial(digitalPinToPinName(Balancebot::PIN_RX_SERIAL_TX), digitalPinToPinName(Balancebot::PIN_RX_SERIAL_RX), NC, NC),
      rx(&rxSerial),
      statusService("19B10000-E8F2-537E-4F6C-D104768A1214"),
      controlService("40760000-8912-48af-9146-b057c465d970"),
      angleCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify),
      targetSpeedCharacteristic("40760001-8912-48af-9146-b057c465d970", BLERead | BLENotify | BLEWriteWithoutResponse),
      targetRotationCharacteristic("40760002-8912-48af-9146-b057c465d970", BLERead | BLENotify | BLEWriteWithoutResponse),
      usePositionHoldCharacteristic("40760003-8912-48af-9146-b057c465d970", BLERead | BLENotify | BLEWriteWithoutResponse),
      isEnabledCharacteristic("40760004-8912-48af-9146-b057c465d970", BLERead | BLENotify | BLEWriteWithoutResponse)
{
    // configure angle hold pid
    anglePid.SetMode(AUTOMATIC);
    anglePid.SetOutputLimits(-30, 30);
    anglePid.SetSampleTime(10);

    // configure position hold pid
    positionPid.SetMode(AUTOMATIC);
    positionPid.SetOutputLimits(-4, 4);
    positionPid.SetSampleTime(10);
}

void Balancebot::setup()
{
    // give some time to attach serial and see the startup logs
    delay(3000);

    // setup everything
    setupPins();
    setupMpu();
    setupBluetooth();
    setupOdrive();
    setupReceiver();

    // transition to balancing state
    setState(BalancebotState::Balancing);
}

void Balancebot::setupPins()
{
    // we're using the built-in led to show bluetooth connection state
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

    // TODO: implement storing offsets in eeprom
    // these values will be shown in the console when performing IMU calibration via RC remote
    mpu.setGyroOffsets(-8.4281, -31.4373, -17.5373);
    mpu.setAccOffsets(0.0255, -0.0104, -0.1493);
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

    // report robot initial state via ble characteristics
    reportAllState();

    // start advertising
    BLE.advertise();

    Serial << " done!" << '\n';
}

void Balancebot::setupOdrive()
{
    Serial << "Setting up odrive..";

    // setup odrive serial using non-standard baudrate
    // run "odrv0.config.uart_baudrate = 256000" in the odrivetool
    odriveSerial.begin(256000);

    // clear odrive errors
    odriveSerial << "sc\n";

    // TODO: configure axes (currently using configuration stored on odrive)

    Serial << " done!" << '\n';
}

void Balancebot::setupReceiver()
{
    Serial << "Setting up receiver..";

    // we're using the second hardware serial
    rx.Begin(1);

    Serial << " done!" << '\n';
}

void Balancebot::loop(unsigned long dt, unsigned long currentTimeUs)
{
    loopBluetooth(dt, currentTimeUs, loopIndex);
    loopReceiver(dt, currentTimeUs, loopIndex);
    loopRobot(dt, currentTimeUs, loopIndex);

    loopIndex++;
}

void Balancebot::loopBluetooth(unsigned long dt, unsigned long currentTimeUs, int loopIndex)
{
    // skip bluetooth if rc receiver is connected
    if (isReceiverConnected)
    {
        return;
    }

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

    // attempt to get central device
    BLEDevice central = BLE.central();

    if (central)
    {
        // handle target speed updates
        if (targetSpeedCharacteristic.written())
        {
            targetSpeed = targetSpeedCharacteristic.value();

            targetSpeedCharacteristic.writeValue(targetSpeed);

            Serial << "Target speed changed to " << targetSpeed << '\n';
        }

        // handle target rotation updates
        if (targetRotationCharacteristic.written())
        {
            targetRotation = targetRotationCharacteristic.value();

            targetRotationCharacteristic.writeValue(targetRotation);

            Serial << "Target rotation changed to " << targetRotation << '\n';
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
void Balancebot::loopReceiver(unsigned long dt, unsigned long currentTimeUs, int loopIndex)
{
    // check if receiver is ready to provide data
    if (rx.Read())
    {
        // fetch receiver data
        rxData = rx.ch();

        // display receiver data if debug mode is enabled (can be toggled from transmitter)
        if (useDebug)
        {
            Serial << "RX: ";

            // display received data
            for (int8_t i = 0; i < bfs::SbusRx::NUM_CH(); i++)
            {
                Serial << rxData[i] << '\t';
            }

            // display lost frames and failsafe data
            Serial << rx.lost_frame() << '\t' << rx.failsafe() << '\n';
        }

        // extract remote data
        if (!rx.failsafe())
        {
            bool wasUsingPositionHold = usePositionHold;
            bool wasEnabled = isEnabled;
            bool wasDebugEnabled = useDebug;
            int lastTargetSpeed = targetSpeed;
            int lastTargetRotation = targetRotation;

            // TODO: implement speed scaling based on potentiometer on the remote
            targetSpeed = map(rxData[0], 172, 1811, -10, 10) * -1;
            targetRotation = map(rxData[1], 172, 1811, -20, 20);
            usePositionHold = rxData[8] > 500;
            isEnabled = rxData[6] < 500;
            useDebug = rxData[4] > 500;
            bool performImuCalibration = rxData[7] > 500;

            // detect target speed change
            if (targetSpeed != lastTargetSpeed)
            {
                Serial << "Target speed changed to " << targetSpeed << '\n';

                targetSpeedCharacteristic.writeValue(targetSpeed);
            }

            // detect target rotation change
            if (targetRotation != lastTargetRotation)
            {
                Serial << "Target rotation changed to " << targetRotation << '\n';

                targetRotationCharacteristic.writeValue(targetRotation);
            }

            // update position hold setpoint if it was just enabled
            if (wasUsingPositionHold != usePositionHold)
            {
                Serial << "Position hold is now " << (usePositionHold ? "enabled" : "disabled") << '\n';

                // update the initial position hold start position once enabled
                if (usePositionHold)
                {
                    updatePositionHoldStartPosition();
                }

                // reset target distance
                targetPosition = 0.0f;
            }

            // detect robot enabled state change
            if (wasEnabled != isEnabled)
            {
                Serial << "Robot is now " << (isEnabled ? "enabled" : "disabled") << '\n';

                // reset target distance
                targetPosition = 0.0f;

                if (isEnabled && usePositionHold)
                {
                    updatePositionHoldStartPosition();
                }
            }

            // detect debug enabled state change
            if (wasDebugEnabled != useDebug)
            {
                Serial << "Debug mode is now " << (useDebug ? "enabled" : "disabled") << '\n';
            }

            // calibrate imu if requested (throttled)
            if (performImuCalibration && currentTimeUs - lastImuCalibrationTimeUs > IMU_CALIBRATION_THROTTLE_US)
            {
                lastImuCalibrationTimeUs = currentTimeUs;

                calibrateImu();
            }

            isReceiverConnected = true;
        }
        else
        {
            // stop the robot if receiver was connected but is not any more
            if (isReceiverConnected)
            {
                Serial << "Receiver connection lost, stopping robot\n";

                targetSpeed = 0;
                targetRotation = 0;
                usePositionHold = true;
                isReceiverConnected = false;

                updatePositionHoldStartPosition();
            }
        }
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
    // increment target distance in position hold mode
    if (usePositionHold)
    {
        targetPosition += (float)targetSpeed * Balancebot::SPEED_DISTANCE_MULTIPLIER * ((float)dt / 1000000.0f);
    }
    else
    {
        targetPosition = 0.0f;
    }

    // TODO: getting odometry is slow..
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

    // log for serial plotter
    // Serial << (currentTimeUs / 1000.0f) << '\t' << angle << '\t' << anglePidSetpoint << '\t' << targetVelocity << '\t' << odometry.leftPosition << '\t' << odometry.rightPosition << '\t' << odometry.position << '\t' << dt << '\n';
}

void Balancebot::setState(BalancebotState newState)
{
    // ignore requesting the same state
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
    // write all ble characteristics
    angleCharacteristic.writeValue(angle);
    targetSpeedCharacteristic.writeValue(targetSpeed);
    targetRotationCharacteristic.writeValue(targetRotation);
    usePositionHoldCharacteristic.writeValue(usePositionHold);
    isEnabledCharacteristic.writeValue(isEnabled);
}

void Balancebot::updatePositionHoldStartPosition()
{
    BalancebotOdometry odometry = getOdometry();

    // this is used to keep the robot at given position
    positionHoldStartPosition = odometry.position;

    Serial << "Updated position hold start position to: " << positionHoldStartPosition << '\n';
}

void Balancebot::calibrateImu()
{
    Serial << "Calculating IMU offsets, keep upright and still..\n";

    // calculate IMU offsets
    mpu.calcOffsets();

    // calibrated imu values can be copies to the setupMpu method
    Serial << "IMU calibrated:\n";
    Serial << "mpu.setGyroOffsets(" << mpu.getGyroXoffset() << ", " << mpu.getGyroYoffset() << ", " << mpu.getGyroZoffset() << ");\n";
    Serial << "mpu.setAccOffsets(" << mpu.getAccXoffset() << ", " << mpu.getAccYoffset() << ", " << mpu.getAccZoffset() << ");\n";
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

    // TODO: calculate real distances using wheel diameter
    return BalancebotOdometry(leftPosition, rightPosition);
}