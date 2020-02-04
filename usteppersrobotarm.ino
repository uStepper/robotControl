#include "robotArmControl.h"

robotArmControl arm;

void setup() {
  arm.begin();
}

void loop() { 
    arm.run(); 
  }
