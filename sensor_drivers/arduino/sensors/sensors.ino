#include <Adafruit_LSM6DS33.h>

struct IMU : public Adafruit_LSM6DS33 {
public:
  IMU() : Adafruit_LSM6DS33() {};
  ~IMU() {};

  void read(uint8_t *buffer) {
    // get raw readings
    Adafruit_BusIO_Register data_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_OUT_TEMP_L, 14);

    data_reg.read(buffer, 14);
  }
};

// IMU
#define IMU_DATA_FREQ   200 // Hz
#define IMU_DATA_PERIOD 1000/IMU_DATA_FREQ // ms
#define IMU_ACCEL_RANGE LSM6DS_ACCEL_RANGE_8_G // +- 8 g
#define IMU_GYRO_RANGE  LSM6DS_GYRO_RANGE_500_DPS // +- 500 deg / s
IMU imu;
unsigned long lastIMUDataTstamp = 0;
uint8_t dataBuffer[14];

unsigned long dataTransmissionStartTime = 0;
bool getConfigCalled = false;

int getAccelRangeNr(lsm6ds_accel_range_t accel_range) {
  switch(accel_range) {
    case LSM6DS_ACCEL_RANGE_2_G:
      return 2;
      break;
    case LSM6DS_ACCEL_RANGE_4_G:
      return 4;
      break;
    case LSM6DS_ACCEL_RANGE_8_G:
      return 8;
      break;
    case LSM6DS_ACCEL_RANGE_16_G:
      return 16;
      break;
  }
}

int getGyroRangeNr(lsm6ds_gyro_range_t gyro_range) {
  switch(gyro_range) {
    case LSM6DS_GYRO_RANGE_125_DPS:
      return 125;
      break;
    case LSM6DS_GYRO_RANGE_250_DPS:
      return 250;
      break;
    case LSM6DS_GYRO_RANGE_500_DPS:
      return 500;
      break;
    case LSM6DS_GYRO_RANGE_1000_DPS:
      return 1000;
      break;
    case LSM6DS_GYRO_RANGE_2000_DPS:
      return 2000;
      break;
  }
}

void setUpError(const char *errorMsg) {
  while (1) {
    Serial.print(F("Error: "));
    Serial.println(errorMsg);
  }
}

void setUpIMU() {
  // Initialize I2C connection with sensor
  if (!imu.begin_I2C())
    setUpError("Failed to find IMU");
  
  // Accelerometer
  imu.setAccelRange(IMU_ACCEL_RANGE);
  imu.setAccelDataRate(LSM6DS_RATE_208_HZ);

  // Gyroscope
  imu.setGyroRange(IMU_GYRO_RANGE);
  imu.setGyroDataRate(LSM6DS_RATE_208_HZ);

  // Disable DRDY (data ready) interrupts, which
  // are set when the FIFO data is ready (we don't use
  // the FIFO).
  imu.configInt1(false, false, false);
  imu.configInt2(false, false, false);
}

bool getMessageFromSerial(const char *msg) {
  int i = 0;
  while (Serial.available() && i < strlen(msg)) {
    if (Serial.read() != msg[i])
      break;
    i++;
    delay(1);
  }
  
  if (i == strlen(msg)) {
    if(Serial.read() == '\r' && Serial.read() == '\n')
      return true;
  }

  return false;
}

void preLoopInit() {
  while (true) {
    if (!getConfigCalled) {
      // Wait for GET_CONFIG message
      if (getMessageFromSerial("GET_CONFIG")) {
        printConfig();
        getConfigCalled = true;
      }
    } else {
      unsigned long tStamp = millis();

      // Wait for BEGIN message
      if (getMessageFromSerial("BEGIN")) {
        dataTransmissionStartTime = tStamp;
        break;
      }
    }
  }
}

void printConfig() {
  Serial.print("Config:");
  Serial.print(IMU_DATA_FREQ);
  Serial.print(",");
  Serial.print(getAccelRangeNr(IMU_ACCEL_RANGE));
  Serial.print(",");
  Serial.print(getGyroRangeNr(IMU_GYRO_RANGE));
  Serial.println();// (Hz, accel data range, gyro data range)
}

void setup() {
  Serial.begin(230400);

  setUpIMU();
  
  preLoopInit();
}

void loop() {
  // Get current time
  unsigned long tStamp = millis();
  
  // If at least an IMU_DATA_PERIOD has elapsed
  // since the last IMU data reading
  if (tStamp >= lastIMUDataTstamp + IMU_DATA_PERIOD) {
    // Update last reading time
    lastIMUDataTstamp += IMU_DATA_PERIOD;
    
    // Read from IMU
    imu.read(dataBuffer);

    // Send data to serial
    Serial.print(tStamp - dataTransmissionStartTime);
    Serial.print(",");

    int i;
    for (i = 13; i >= 0; i--) {
      if (dataBuffer[i] < 16)
        Serial.print(0);
      Serial.print(dataBuffer[i], HEX);
    }
    Serial.println();
  }
}
