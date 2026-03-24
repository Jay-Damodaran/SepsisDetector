#include <Adafruit_LSM6DSO32.h>

#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11
#define LSM_CS 10

#define DELAY 8333

Adafruit_LSM6DSO32 imu_i2c;
Adafruit_LSM6DSO32 imu_spi;

// arrays to hold accelerations and angular velocities
float acc1[3];
float w1[3];
float acc2[3];
float w2[3];

int t0 = micros();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  //SPI.begin();

  if (!imu_i2c.begin_I2C()) {
    Serial.println("I2C IMU not found");
    while (1);
  }
  
  // if (!imu_spi.begin_SPI(LSM_CS)) {
  //   Serial.println("SPI IMU not found");
  //   while (1);
  // }
  // else{
  //   Serial.println("SPI IMU found");
  // }

 // Serial.println("Both IMUs ready");

  imu_i2c.setAccelRange(LSM6DSO32_ACCEL_RANGE_4_G);
  imu_i2c.setAccelDataRate(LSM6DS_RATE_208_HZ);
  //imu_spi.setAccelRange(LSM6DSO32_ACCEL_RANGE_4_G);
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
  Serial.println("Ax,Ay,Az,Amag");
}

void loop() {
  // put your main code here, to run repeatedly:
  // high level adafruit code
  t0 = micros();
  sensors_event_t acc1, gyro1, temp1;
  // sensors_event_t acc2, gyro2, temp2;

  imu_i2c.getEvent(&acc1, &gyro1, &temp1);
  // imu_spi.getEvent(&acc2, &gyro2, &temp2);

  // imu_i2c.readAcceleration(acc1[0], acc1[1], acc1[2]);
  // imu_i2c.readGyroscope(w1[0], w1[1], w1[2]);
  // imu_spi.readAcceleration(acc2[0], acc2[1], acc2[2]);
  // imu_spi.readGyroscope(w2[0], w2[1], w2[2]);

  //Serial.print(millis()); Serial.print(",");
  Serial.print(acc1.acceleration.x); Serial.print(",");
  Serial.print(acc1.acceleration.y); Serial.print(",");
  Serial.print(acc1.acceleration.z); Serial.print(",");
  Serial.println(sqrt(pow(acc1.acceleration.x, 2) + pow(acc1.acceleration.y, 2) + pow(acc1.acceleration.z, 2)));
  // Serial.print(gyro1.gyro.x);   Serial.print(" ");
  // Serial.print(gyro1.gyro.y);   Serial.print(" ");
  // Serial.println(gyro1.gyro.z);
  //Serial.println();
  // Serial.println("SPI Ax Ay Az Gx Gy Gz");
  // Serial.print(acc2[0]); Serial.print(" ");
  // Serial.print(acc2[1]); Serial.print(" ");
  // Serial.print(acc2[2]); Serial.print(" ");
  // Serial.print(w2[0]);   Serial.print(" ");
  // Serial.print(w2[1]);   Serial.print(" ");
  // Serial.println(w2[2]); 
  t0 = micros() - t0;
  delayMicroseconds(DELAY - t0); // 120 Hz fs
}
