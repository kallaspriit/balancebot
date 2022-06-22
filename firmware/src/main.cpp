// includes
#include <HardwareSerial.h>
// #include <SoftwareSerial.h>
#include <ODriveArduino.h>
// Printing with stream operator helper functions
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

////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////

// Below are some sample configurations.
// You can comment out the default Teensy one and uncomment the one you wish to use.
// You can of course use something different if you like
// Don't forget to also connect ODrive GND to Arduino GND.

// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX
// pin 1: TX - connect to ODrive RX
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
HardwareSerial &odrive_serial = Serial1;

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
// HardwareSerial& odrive_serial = Serial1;

// Arduino without spare serial ports (such as Arduino UNO) have to use software serial.
// Note that this is implemented poorly and can lead to wrong data sent or read.
// pin 8: RX - connect to ODrive TX
// pin 9: TX - connect to ODrive RX
// SoftwareSerial odrive_serial(8, 9);

// ODrive object
ODriveArduino odrive(odrive_serial);

const int PIN_DEBUG_LED = LED_BUILTIN;

void setup()
{
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

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

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  // for (int axis = 0; axis < 2; ++axis)
  // {
  //   odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 10.0f << '\n';
  //   odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';
  //   // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  // }

  Serial.println("Ready!");
  // Serial.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");
  Serial.println("Send the character 's' to exectue test move");
  Serial.println("Send the character 'b' to read bus voltage");
  Serial.println("Send the character 'p' to read motor positions in a 10s loop");
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

    // Sinusoidal test move
    if (c == 's')
    {
      Serial.println("Executing test move..");

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

    // Read bus voltage
    if (c == 'b')
    {
      odrive_serial << "r vbus_voltage\n";

      Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
    }

    // print motor positions in a 10s loop
    if (c == 'p')
    {
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