
#include <SPI.h>
#include<RF24.h>
#include <Servo.h>

int JL_x;
int JL_y;
int JR_x;
int JR_y;
int tSwitch[] = {0, 0, 0};

int trottle = 0;
int pitch = 0;
int yaw = 0;
int leftWing = 0;
int rightWing = 0;

int max_dgrees = 180;
int min_dgrees = 0;
int middle_dgrees = 90;


int brushlessMotorPin = 6;
int pitchMotorPin = A0;
int yawMotorPin = 5;
int leftWingMotorPin = 3;
int rightWingMotorPin = A1;

RF24 radio (9, 10);
byte address[][6] = {"ADR00001"};
int data[8];

Servo brushlesMotor;
Servo pitchMotor;
Servo yawMotor;
Servo leftWingMotor;
Servo rightWingMotor;

void setup() {

  Serial.begin(9600);

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(124);
  radio.openReadingPipe(1, address[0]);
  radio.startListening();

  brushlesMotor.attach(brushlessMotorPin);
  pitchMotor.attach(pitchMotorPin);
  yawMotor.attach(yawMotorPin);
  leftWingMotor.attach(leftWingMotorPin);
  rightWingMotor.attach(rightWingMotorPin);

  brushlesMotor.writeMicroseconds(1000);
}

void loop() {
  if (radio.available()) {

    radio.read(&data, sizeof(data));

    for (int i = 0; i < 3; i++) {
      tSwitch[i] =  data[i];
      trottle = map(data[3], 0, 550, 1000, 2000);
      JL_x = data[4];
      JL_y = data[5];
      JR_x = data[6];
      JR_y = data[7];

    }

    upDown();
    leftRight();
    rawing();
    printing();

    if (tSwitch[0] == 1) {
      brushlesMotor.writeMicroseconds(trottle);
      pitchMotor.write(180 - pitch);
      yawMotor.write(180 - yaw);
      leftWingMotor.write(leftWing);
      rightWingMotor.write(rightWing);

    }
    delay(50);
  } else {
    Serial.println("error");

  }
}

void upDown() {
  if (JR_y < 460) {
    yaw = map(JR_y, 460, 0, middle_dgrees, min_dgrees );

  }
  else if (JR_y > 560) {
    yaw = map(JR_y, 540, 1023, middle_dgrees, max_dgrees);
  } else {
    yaw = middle_dgrees;
  }
}

void leftRight() {
  if (JR_x < 460) {
    pitch = map(JR_x, 460, 0, middle_dgrees, max_dgrees);
  }
  else if (JR_x > 560) {
    pitch = map(JR_x, 560, 1023, middle_dgrees, min_dgrees);
  } else {
    pitch = middle_dgrees;
  }
}

void rawing() {
  if (yaw < middle_dgrees) {
    rightWing = yaw;
    leftWing = map(yaw, middle_dgrees, min_dgrees, middle_dgrees, max_dgrees);
  } else if (yaw > middle_dgrees) {
    rightWing = yaw;
    leftWing = map(yaw, middle_dgrees, max_dgrees, middle_dgrees, min_dgrees);
  } else {
    leftWing = pitch;
    rightWing = pitch;


  }
}

void printing() {

  Serial.print("trottle : "); Serial.print(trottle); Serial.print("   ");
  Serial.print("pitch : "); Serial.print(pitch); Serial.print("   ");
  Serial.print("yaw : "); Serial.print(yaw); Serial.print("   ");
  Serial.print("left wing : "); Serial.print(leftWing); Serial.print("   ");
  Serial.print("right wing : "); Serial.print(rightWing); Serial.print("   ");

  Serial.println();
}

