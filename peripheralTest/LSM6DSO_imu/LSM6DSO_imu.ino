#include <Adafruit_LSM6DSO32.h>

#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11
#define LSM_CS 10

#define DELAY 11111

Adafruit_LSM6DSO32 imu_i2c;
Adafruit_LSM6DSO32 imu_i2c2;

// arrays to hold accelerations and angular velocities
float acc1[3];
float acc2[3];

int t0 = micros();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  //SPI.begin();

  if (!imu_i2c.begin_I2C(0x6A)) {
    Serial.println("I2C IMU not found");
    while (1);
  }

  if (!imu_i2c2.begin_I2C(0x6B)) {
    Serial.println("2nd I2C IMU not found");
    while (1);
  }
  
  // if (!imu_spi.begin_SPI(LSM_CS)) {
  //   Serial.println("SPI IMU not found");
  //   while (1);
  // }
  // else{
  //   Serial.println("SPI IMU found");
  // }

  Serial.println("Both IMUs ready");

  imu_i2c.setAccelRange(LSM6DSO32_ACCEL_RANGE_4_G);
  imu_i2c.setAccelDataRate(LSM6DS_RATE_208_HZ);
  imu_i2c2.setAccelRange(LSM6DSO32_ACCEL_RANGE_4_G);
  imu_i2c2.setAccelDataRate(LSM6DS_RATE_208_HZ);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(800);
  digitalWrite(LED_BUILTIN, LOW);
  delay(800);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(800);
  digitalWrite(LED_BUILTIN, LOW);
  delay(800);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(800);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("start");
  Serial.println("Ax,Ay,Az,Amag,Ax2,Ay2,Az2,Amag2");
}


void loop() {
  // put your main code here, to run repeatedly:
  // high level adafruit code
  t0 = micros();
  sensors_event_t acc1, gyro1, temp1;
  sensors_event_t acc2, gyro2, temp2;
  imu_i2c.getEvent(&acc1, &gyro1, &temp1);
  imu_i2c2.getEvent(&acc2, &gyro2, &temp2);

  Serial.print(acc1.acceleration.x); Serial.print(",");
  Serial.print(acc1.acceleration.y); Serial.print(",");
  Serial.print(acc1.acceleration.z); Serial.print(",");
  Serial.print(sqrt(pow(acc1.acceleration.x, 2) + pow(acc1.acceleration.y, 2) + pow(acc1.acceleration.z, 2))); Serial.print(",");
  Serial.print(acc2.acceleration.x); Serial.print(",");
  Serial.print(acc2.acceleration.y); Serial.print(",");
  Serial.print(acc2.acceleration.z); Serial.print(",");
  Serial.println(sqrt(pow(acc2.acceleration.x, 2) + pow(acc2.acceleration.y, 2) + pow(acc2.acceleration.z, 2)));
  t0 = micros() - t0;
  delayMicroseconds(DELAY - t0); // 90 Hz fs
}
