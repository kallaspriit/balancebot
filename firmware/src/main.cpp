#include "Balancebot.hpp"
#include "stream_operators.hpp"

// pids for the angle controller keeping the robot upright
const double ANGLE_PID_P = 2.0;
const double ANGLE_PID_I = 15.0;
const double ANGLE_PID_D = 0.01;

// pids for the position hold controller keeping robot in the same spot
const double POSITION_PID_P = 0.3;
const double POSITION_PID_I = 0.0;
const double POSITION_PID_D = 0.2;

// setup configuration and robot
BalancebotConfig config = BalancebotConfig(
    ANGLE_PID_P,
    ANGLE_PID_I,
    ANGLE_PID_D,
    POSITION_PID_P,
    POSITION_PID_I,
    POSITION_PID_D);
Balancebot balanceBot = Balancebot(config);

// runtime info
unsigned long lastStepTimeUs = 0;

// run once to setup everything
void setup()
{
  // open serial connection to pc
  Serial.begin(115200);

  // setup the robot
  balanceBot.setup();

  // set initial last step time 10ms in the past
  lastStepTimeUs = micros() - 10 * 1000;
}

void loop()
{
  // calculate loop delta time
  unsigned long currentTimeUsUs = micros();
  unsigned long dt = currentTimeUsUs - lastStepTimeUs;

  // update the robot
  balanceBot.loop(dt, currentTimeUsUs);

  lastStepTimeUs = currentTimeUsUs;
}
