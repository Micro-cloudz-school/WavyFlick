#include <Wire.h>
#include <IRremote.h>

const int MPU_ADDR = 0x68;

int16_t AccX, AccY, AccZ;
int16_t GyroX, GyroY, GyroZ;

float roll = 0.0;
float pitch = 0.0;
float alpha = 0.96;

float mappedRoll = 0.0;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

unsigned long timer;

const int irPin = 2;

int upstat = 0;
unsigned long up_timestamp = 0;

void setup() {
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    IrSender.begin(irPin);

    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(9600);
    timer = micros();
}

void loop() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU_ADDR, 14);
    if (Wire.available() < 14) return;

    AccX = Wire.read() << 8 | Wire.read();
    AccY = Wire.read() << 8 | Wire.read();
    AccZ = Wire.read() << 8 | Wire.read();

    Wire.read(); Wire.read();

    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

    float dt = (micros() - timer) / 1000000.0;
    timer = micros();

    float denom = sqrt(AccY * AccY + AccZ * AccZ);
    if (denom == 0) denom = 0.00001;

    float accRoll = atan2(AccY, AccZ) * 180 / PI;
    float accPitch = atan2(-AccX, denom) * 180 / PI;

    float gyroRollRate = GyroX / 131.0;
    float gyroPitchRate = GyroY / 131.0;

    roll = alpha * (roll + gyroRollRate * dt) + (1 - alpha) * accRoll;
    pitch = alpha * (pitch + gyroPitchRate * dt) + (1 - alpha) * accPitch;

  mappedRoll = mapFloat(roll, -90.0, 90.0, -20.0, 20.0);
  mappedRoll = constrain(mappedRoll, -20.0, 20.0);

    Serial.println(mappedRoll);  

    // If roll is between 50 and 100 → upstat = 1
if (roll > 40 && roll < 100) {
    if (upstat == 0) up_timestamp = millis();  // just turned 1
    upstat = 1;
} 
else {
    // upstat will become 0 — check how long it was 1
    if (upstat == 1) {
        if (millis() - up_timestamp <= 1000) {
            // it turned to zero within 1 second
            //Serial.println("upstat dropped within 1 second!");
            IrSender.sendNEC(0xF803, 0x1, 2);
        }
    }
    upstat = 0;
}


    delay(10);
}
