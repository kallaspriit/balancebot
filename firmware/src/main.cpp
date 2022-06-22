#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include <MPU6050_light.h>
#include <Wire.h>

// printing with stream operator helper functions
template <class T>
inline Print &operator<<(Print &obj, T arg)
{
  obj.print(arg);
  return obj;
}
template <>
inline Print &operator<<(Print &obj, float arg)
{
  obj.print(arg, 4);
  return obj;
}

HardwareSerial &odrive_serial = Serial1;
// HardwareSerial &console = Serial;

ODriveArduino odrive(odrive_serial);
MPU6050 mpu(Wire);

const int PIN_DEBUG_LED = LED_BUILTIN;

void setup()
{
  // Serial to PC
  Serial.begin(115200);

  // Setup debug led
  pinMode(PIN_DEBUG_LED, OUTPUT);

  // wait for Arduino Serial Monitor to open
  while (!Serial)
  {
    digitalWrite(PIN_DEBUG_LED, HIGH);
    delay(200);
    digitalWrite(PIN_DEBUG_LED, LOW);
    delay(200);
  }

  // keep the debug led lit
  digitalWrite(PIN_DEBUG_LED, HIGH);

  Serial.print("Searching for MPU6050..");

  Wire.begin();

  if (mpu.begin() != 0)
  {
    Serial.println(" failed!");

    while (1)
    {
      digitalWrite(PIN_DEBUG_LED, HIGH);
      delay(100);
      digitalWrite(PIN_DEBUG_LED, LOW);
      delay(100);
    }
  }
  Serial.println(" done!");

  Serial.print(F("Calculating IMU offsets, keep still.. "));
  delay(1000);
  mpu.calcOffsets();
  Serial.println(" done!");

  Serial.print("Setting up odrive..");

  odrive_serial.begin(115200);

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  // for (int axis = 0; axis < 2; ++axis)
  // {
  //   odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 10.0f << '\n';
  //   odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';
  //   // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  // }

  Serial.println(" done!");
  Serial.println();

  Serial.println("Send the character 'a' to let IMU control motor speed");
  Serial.println("Send the character 'b' to read bus voltage");
  Serial.println("Send the character 'p' to read motor positions in a 10s loop");
  Serial.println("Send the character 't' to exectute test move");
}

void loop()
{

  if (Serial.available())
  {
    char c = Serial.read();

    // Run calibration sequence
    // if (c == '0' || c == '1')
    // {
    //   int motornum = c - '0';
    //   int requested_state;

    //   requested_state = AXIS_STATE_MOTOR_CALIBRATION;
    //   Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
    //   if (!odrive.run_state(motornum, requested_state, true))
    //     return;

    //   requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
    //   Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
    //   if (!odrive.run_state(motornum, requested_state, true, 25.0f))
    //     return;

    //   requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
    //   Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
    //   if (!odrive.run_state(motornum, requested_state, false /*don't wait*/))
    //     return;
    // }

    // use IMU angle to control motor velocity
    if (c == 'a')
    {
      Serial.println("Using IMU angle to control the motor for 10 seconds");

      // use velocity control with passthrough
      odrive_serial << "w axis0.controller.config.control_mode " << CONTROL_MODE_VELOCITY_CONTROL << '\n';
      odrive_serial << "w axis0.controller.config.input_mode " << INPUT_MODE_PASSTHROUGH << '\n';

      static const unsigned long duration = 10000;
      unsigned long start = millis();

      while (millis() - start < duration)
      {
        mpu.update();

        // get X angle and calculate velocity (limited)
        float angle = mpu.getAngleX();
        float velocity = max(min(angle / 2.0f, 10.0f), -10.0f);

        // odrive_serial << "w axis0.controller.input_vel " << velocity << '\n';

        odrive.SetVelocity(0, velocity);

        Serial << "Angle: " << angle << ", velocity: " << velocity << '\n';
      }

      odrive.SetVelocity(0, 0.0f);

      Serial.println("Using IMU angle to control the motor done");
    }

    // read bus voltage
    if (c == 'b')
    {
      odrive_serial << "r vbus_voltage\n";

      Serial << "Odrive voltage: " << odrive.readFloat() << '\n';
    }

    // print motor positions in a 10s loop
    if (c == 'p')
    {
      Serial.println("Tracking position for 10 seconds");

      static const unsigned long duration = 10000;
      unsigned long start = millis();

      while (millis() - start < duration)
      {
        for (int motor = 0; motor < 2; ++motor)
        {
          Serial << odrive.GetPosition(motor) << '\t';
        }

        Serial << '\n';
      }

      Serial.println("Tracking position complete");
    }

    // test move
    if (c == 't')
    {
      Serial.println("Executing test move");

      // use position control with trapecoidal trajectory
      odrive_serial << "w axis0.controller.config.control_mode " << CONTROL_MODE_POSITION_CONTROL << '\n';
      odrive_serial << "w axis0.controller.config.input_mode " << INPUT_MODE_TRAP_TRAJ << '\n';

      Serial.println("  > going to zero");
      odrive.SetPosition(0, 0);
      delay(1000);

      Serial.println("  > going five turns forward");
      odrive.SetPosition(0, 5);
      delay(2000);

      Serial.println("  > going back to zero");
      odrive.SetPosition(0, 0);
      delay(2000);

      // for (float ph = 0.0f; ph < PI * 2; ph += 0.01f)
      // {
      //   float pos_m0 = 1.0f * cos(ph);
      //   // float pos_m1 = 2.0f * sin(ph);

      //   odrive.SetPosition(0, pos_m0);
      //   // odrive.SetPosition(1, pos_m1);

      //   delay(5);
      // }

      Serial.println("Test move done!");
    }
  }
}

// #include <Arduino.h>

// void setup()
// {
//   pinMode(LED_BUILTIN, OUTPUT);
// }

// void loop()
// {
//   digitalWrite(LED_BUILTIN, HIGH);
//   delay(50);
//   digitalWrite(LED_BUILTIN, LOW);
//   delay(50);
// }