#define dirPin 4
#define stepPin 5
#define stepsPerRevolution 1600
void setup() {
  Serial.begin(115200);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(A5, INPUT);
  while (digitalRead(A5))
  {
    drive_up(1);
  }
}
byte head;
byte addr;
byte start;
byte package[7];
uint8_t pair = 1;
long int data_z = 0;
long int pose_z = 0;
long int goal_z = 0;
long int orentation_gripper = 0;
long int position_gripper = 0;
long int checksum;
void clearbuffer() {
  for (int i = 0; i < sizeof(package); i++) {
    package[i] = 0;
  }
}
void loop() {
  if (Serial.available() >= 7) {
    for (int i = 0; i < 7; i++) {
      package[i] = Serial.read();
    }
    if (package[0] == 0xBD)
    {
      if (package[1] == 0x61)
      {
        checksum = ((package[1] + package[2] + package[3] + package[4] + package[5]) & 0xFF );
        checksum = ~(checksum) & 0xFF;

        if (checksum == package[6]) {
          data_z = (package[2] << 8) | (package[3]);
          orentation_gripper = package[4];
          position_gripper = package[5];
        }
      }
    }

    //      Serial.write(package[6]);

    //        Serial.print(package[6]);

    if (data_z !=  pose_z)
    {
      goal_z = pose_z - data_z ;
      Serial.println(goal_z);
      Serial.println(data_z);
      Serial.println(pose_z);
      if (goal_z > 0)
      {
        drive_up((abs(pose_z)-abs(data_z)) * 42);
        pose_z = data_z;
      }
      else if (goal_z < 0)
      {
        drive_down((abs(pose_z)-abs(data_z)) * -42);
        pose_z = data_z;
      }
      
    }
    
  }
}

void drive_up(int r) {
  for (int i = 0; i < r; i++) {
    digitalWrite(dirPin, HIGH);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(800);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(800);
  }
}


void drive_down(int r) {
  for (int i = 0; i < r; i++) {
    digitalWrite(dirPin, LOW);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(800);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(800);
  }
}

int map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
