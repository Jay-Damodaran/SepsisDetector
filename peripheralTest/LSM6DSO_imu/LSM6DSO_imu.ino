#include <Adafruit_LSM6DSOX.h>

#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11
#define LSM_CS 10

Adafruit_LSM6DSOX imu_i2c;
Adafruit_LSM6DSOX imu_spi;

// arrays to hold accelerations and angular velocities
float acc1[3];
float w1[3];
float acc2[3];
float w2[3];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);

  // ---- I2C IMU ----
  if (!imu_i2c.begin_I2C()) {
    Serial.println("I2C IMU not found");
    while (1);
  }

  // ---- SPI IMU ----
  if (!imu_spi.begin_SPI(LSM_CS)) {
    Serial.println("SPI IMU not found");
    while (1);
  }

  Serial.println("Both IMUs ready");
  
  // sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (imu_i2c.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  Serial.print("Gyro range set to: ");
  switch (imu_i2c.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DSOX
  }

  Serial.print("Accelerometer data rate set to: ");
  switch (imu_i2c.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  Serial.print("Gyro data rate set to: ");
  switch (imu_i2c.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  imu_i2c.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  imu_spi.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  imu_i2c.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  imu_spi.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
}

void loop() {
  // put your main code here, to run repeatedly:
  // high level adafruit code
  // sensors_event_t acc1, gyro1, temp1;
  // sensors_event_t acc2, gyro2, temp2;

  // imu_i2c.getEvent(&acc1, &gyro1, &temp1);
  // imu_spi.getEvent(&acc2, &gyro2, &temp2);

  // Serial.print("I2C AX: ");
  // Serial.print(acc1.acceleration.x);
  // Serial.print(" | SPI AX: ");
  // Serial.println(acc2.acceleration.x);

  imu_i2c.readAcceleration(acc1[0], acc1[1], acc1[2]);
  imu_i2c.readGyroscope(w1[0], w1[1], w1[2]);
  imu_spi.readAcceleration(acc2[0], acc2[1], acc2[2]);
  imu_spi.readGyroscope(w2[0], w2[1], w2[2]);

  Serial.println("I2C Ax Ay Az Gx Gy Gz");
  Serial.print(acc1[0]); Serial.print(" ");
  Serial.print(acc1[1]); Serial.print(" ");
  Serial.print(acc1[2]); Serial.print(" ");
  Serial.print(w1[0]);   Serial.print(" ");
  Serial.print(w1[1]);   Serial.print(" ");
  Serial.println(w1[2]);
  Serial.println();
  Serial.println("SPI Ax Ay Az Gx Gy Gz");
  Serial.print(acc2[0]); Serial.print(" ");
  Serial.print(acc2[1]); Serial.print(" ");
  Serial.print(acc2[2]); Serial.print(" ");
  Serial.print(w2[0]);   Serial.print(" ");
  Serial.print(w2[1]);   Serial.print(" ");
  Serial.println(w2[2]); 

  delay(100);
}
