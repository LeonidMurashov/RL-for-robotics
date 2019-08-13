#include <Wire.h>
#include <Multiservo.h>

const int SERVO_COUNT = 18;
const int SMALLEST_ANGLE = 30;
const int BIGGEST_ANGLE = 150;

Multiservo servo[SERVO_COUNT];
int servo_positions[SERVO_COUNT];

void setup() {
  Wire.begin();
  Serial.begin(9600);

  for (int i = 0; i < SERVO_COUNT; ++i) {
    servo[i].attach(i);
  }
}

void loop() {  
  // If msg arrived
  if (Serial.available() >= SERVO_COUNT) {
    for (int i = 0; i < SERVO_COUNT; ++i) {
      servo_positions[i] = Serial.read();
    }

    // Drive or kill servos
    for (int i = 0; i < SERVO_COUNT; ++i) {
      if(servo_positions[i] <= 180) {
        servo[i].attach(i); // Not shure
        int angle = constrain(servo_positions[i], SMALLEST_ANGLE, BIGGEST_ANGLE);
        servo[i].write(angle);
      } else {
        // Kill servos
        servo[i].detach();
      }
    }
  }
}
