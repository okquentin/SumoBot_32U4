#include <Arduino.h>
#include <Sumo.h>

// #define LOG_SERIAL

void setup() {
  sensors.initFiveSensors();

  // Initialize the Wire library and join the I2C bus as a master
  Wire.begin();

  // Initialize accelerometer
  acc.init();
  acc.enableDefault();

#ifdef LOG_SERIAL
  acc.getLogHeader();
#endif

  randomSeed((unsigned int) millis());

  // Uncomment if necessary to correct motor directions:
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

  ledYellow(1);
  buzzer.playMode(PLAY_AUTOMATIC);
  waitForButtonAndCountDown(false);
}

void loop() {
  if (button.isPressed()) {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    waitForButtonAndCountDown(true);
    display.print("Driving");
  }

  loop_start_time = millis();
  acc.readAcceleration(loop_start_time);
  sensors.read(sensor_values);

  // if ((_forwardSpeed == FullSpeed) && (loop_start_time - full_speed_start_time > FULL_SPEED_DURATION_LIMIT)) {
  //   setForwardSpeed(SustainedSpeed);
  // }

  if (sensor_values[0] < QTR_THRESHOLD) {
    #ifdef LOG_SERIAL
    Serial.print("Left Sensor Line Detected");
    Serial.println();
    #else
    // if leftmost sensor detects line, reverse and turn to the right
    turn(RIGHT, true);
    #endif
  }
  else if (sensor_values[NUM_SENSORS - 1] < QTR_THRESHOLD) {
    #ifdef LOG_SERIAL
    Serial.print("Right Sensor Line Detected");
    Serial.println();
    #else
    // if rightmost sensor detects line, reverse and turn to the left
    turn(LEFT, true);
    #endif
  }
  else {  // otherwise, go straight
    int speed = getForwardSpeed();

    // when the robot makes contact with another robot
    if (check_for_contact()) {
      on_contact_made();
      // pulseMotors();
    }
    else
      motors.setSpeeds(speed, speed);
    
    // if the contact time is greater than 4 seconds update the display
    if(millis() - contactTime >= 4000 && displayed != true){
     display.clear();
     display.print("Driving");
     displayed = true;
    }

  }
}



