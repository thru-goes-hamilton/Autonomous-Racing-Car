#include <Adafruit_VL53L0X.h>
#include <Wire.h>

// Define the I2C Wire object
TwoWire I2C_BUS = TwoWire(0);

// Sensor objects
Adafruit_VL53L0X sensor1;
Adafruit_VL53L0X sensor2;
Adafruit_VL53L0X sensor3;

// Sensor configurations
const uint8_t SENSOR1_ADDR = 0x30;
const uint8_t SENSOR2_ADDR = 0x31;
const uint8_t SENSOR3_ADDR = 0x32;

const uint8_t SENSOR1_XSHUT_PIN = 27;
const uint8_t SENSOR2_XSHUT_PIN = 26;
const uint8_t SENSOR3_XSHUT_PIN = 25;

void resetSensors() {
  // Set all XSHUT pins low to shutdown sensors
  digitalWrite(SENSOR1_XSHUT_PIN, LOW);
  digitalWrite(SENSOR2_XSHUT_PIN, LOW);
  digitalWrite(SENSOR3_XSHUT_PIN, LOW);
  delay(10);

  // Set XSHUT pins high to enable sensors
  digitalWrite(SENSOR1_XSHUT_PIN, HIGH);
  digitalWrite(SENSOR2_XSHUT_PIN, HIGH);
  digitalWrite(SENSOR3_XSHUT_PIN, HIGH);
  delay(10); // Give time for sensors to wake up
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA, SCL pins for I2C

  // Initialize XSHUT pins as outputs
  pinMode(SENSOR1_XSHUT_PIN, OUTPUT);
  pinMode(SENSOR2_XSHUT_PIN, OUTPUT);
  pinMode(SENSOR3_XSHUT_PIN, OUTPUT);

  resetSensors(); // Reset and enable sensors

  // Initialize sensors
  if (!sensor1.begin(SENSOR1_ADDR, &I2C_BUS)) {
    Serial.println("Failed to initialize sensor1");
  }
  if (!sensor2.begin(SENSOR2_ADDR, &I2C_BUS)) {
    Serial.println("Failed to initialize sensor2");
  }
  if (!sensor3.begin(SENSOR3_ADDR, &I2C_BUS)) {
    Serial.println("Failed to initialize sensor3");
  }
}

void loop() {
  // Read distance measurements from all sensors
  uint16_t distance1 = sensor1.readRange();
  uint16_t distance2 = sensor2.readRange();
  uint16_t distance3 = sensor3.readRange();

  // Check for timeouts
  bool timeout1 = sensor1.timeoutOccurred();
  bool timeout2 = sensor2.timeoutOccurred();
  bool timeout3 = sensor3.timeoutOccurred();

  // Print distance measurements and timeout status
  Serial.print("Sensor1: ");
  Serial.print(distance1);
  Serial.print(" mm");
  if (timeout1) Serial.print(" (Timeout)");
  Serial.print("\tSensor2: ");
  Serial.print(distance2);
  Serial.print(" mm");
  if (timeout2) Serial.print(" (Timeout)");
  Serial.print("\tSensor3: ");
  Serial.print(distance3);
  Serial.print(" mm");
  if (timeout3) Serial.print(" (Timeout)");
  Serial.println();

  delay(100); // Delay between readings (adjust as needed)
}
