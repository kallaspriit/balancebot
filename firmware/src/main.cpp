#include "BalanceBot.hpp"
#include "stream_operators.hpp"

// configuration
const int TARGET_UPDATE_FREQUENCY = 100;
const int LOOP_DURATION = 1000 / TARGET_UPDATE_FREQUENCY;

// dependency instances
BalanceBot balanceBot;

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
