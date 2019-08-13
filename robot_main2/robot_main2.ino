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

int servo_iterator = 0;
void loop() {
  // If msg arrived
  if (Serial.available()) {
    int symbol = Serial.read();

    // Drive or kill servos
    if(symbol <= 180) {
      servo[servo_iterator].attach(servo_iterator); // Not shure
      int angle = constrain(symbol, SMALLEST_ANGLE, BIGGEST_ANGLE);
      servo[servo_iterator].write(angle);
    } else {
      // Kill servos
      servo[servo_iterator].detach();
    }
    servo_iterator = (servo_iterator + 1) % SERVO_COUNT;
  }
}
