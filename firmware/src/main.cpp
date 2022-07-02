#include "Balancebot.hpp"
#include "stream_operators.hpp"

// configuration
const int TARGET_UPDATE_FREQUENCY = 100;
const int LOOP_DURATION = 1000 / TARGET_UPDATE_FREQUENCY;
// const double ANGLE_PID_P = 0.3;
// const double ANGLE_PID_I = 4.0;
// const double ANGLE_PID_D = 0.005;
const double ANGLE_PID_P = 0.6;
const double ANGLE_PID_I = 8.0;
const double ANGLE_PID_D = 0.01;

// const double POSITION_PID_P = 0.5;
// const double POSITION_PID_I = 0.0;
// const double POSITION_PID_D = 0.1;
const double POSITION_PID_P = 1.0;
const double POSITION_PID_I = 0.0;
const double POSITION_PID_D = 1.0;

// dependency instances
BalancebotConfig config = BalancebotConfig(
    ANGLE_PID_P,
    ANGLE_PID_I,
    ANGLE_PID_D,
    POSITION_PID_P,
    POSITION_PID_I,
    POSITION_PID_D);
Balancebot balanceBot = Balancebot(config);

// runtime info
unsigned long lastStepTime = 0;

// run once to setup everything
void setup()
{
  // open serial connection to pc
  Serial.begin(115200);

  // setup the robot
  balanceBot.setup();

  // set initial last step time 10ms in the past
  lastStepTime = millis() - 10;
}

void loop()
{
  // calculate loop delta time
  unsigned long currentTime = millis();
  unsigned long dt = currentTime - lastStepTime;

  // update the robot
  balanceBot.loop(dt, currentTime);

  // calculate time to sleep to achieve target update frequency
  // int sleepTimeMs = LOOP_DURATION - (int)dt;

  // // sleep to  achieve target update frequency
  // if (sleepTimeMs > 0)
  // {
  //   delay(sleepTimeMs);
  // }

  lastStepTime = currentTime;
}
