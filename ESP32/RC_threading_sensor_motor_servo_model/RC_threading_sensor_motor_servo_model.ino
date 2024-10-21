#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "model.h" // Include the TensorFlow Lite model

// RemoteXY configuration
#define REMOTEXY_MODE__WIFI_POINT
#include <RemoteXY.h>

#define REMOTEXY_WIFI_SSID "RemoteXY"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377

// RemoteXY interface configuration
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] = {
  255,2,0,5,0,137,0,19,0,0,0,0,31,2,106,200,200,84,1,1,
  7,0,10,248,41,57,57,125,18,49,49,48,120,1,31,79,70,70,0,31,
  79,78,0,74,22,43,21,24,17,11,40,10,12,2,30,37,64,48,0,74,
  5,81,21,24,13,45,29,9,12,2,30,37,64,48,0,74,37,83,21,24,
  65,11,40,10,12,2,30,37,64,48,0,74,9,143,21,24,49,45,28,9,
  12,2,30,37,64,48,0,74,35,145,21,24,84,45,28,9,12,2,30,37,
  64,48,0,10,5,143,57,57,50,64,15,15,48,4,26,31,79,78,0,31,
  79,70,70,0
};

struct {
  uint8_t emergency_button;
  uint8_t sensor_button;
  uint8_t motor_val;
  uint8_t left_sensor;
  uint8_t servo_val;
  uint8_t middle_sensor;
  uint8_t right_sensor;
  uint8_t connect_flag;
} RemoteXY;
#pragma pack(pop)

// Sensor and Servo configuration
Servo myservo;

#define SENSOR1_WIRE Wire
#define SENSOR2_WIRE Wire
#define SENSOR3_WIRE Wire

typedef enum {
  RUN_MODE_DEFAULT = 1,
  RUN_MODE_ASYNC,
  RUN_MODE_GPIO,
  RUN_MODE_CONT
} runmode_t;

runmode_t run_mode = RUN_MODE_ASYNC;

typedef struct {
  Adafruit_VL53L0X *psensor;
  TwoWire *pwire;
  int id;
  int shutdown_pin;
  int interrupt_pin;
  Adafruit_VL53L0X::VL53L0X_Sense_config_t sensor_config;
  uint16_t range;
  uint8_t sensor_status;
} sensorList_t;

Adafruit_VL53L0X sensor1;
Adafruit_VL53L0X sensor2;
Adafruit_VL53L0X sensor3;

sensorList_t sensors[] = {
  {&sensor1, &SENSOR1_WIRE, 0x30, 26, 32, Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE, 0, 0},
  {&sensor2, &SENSOR2_WIRE, 0x31, 27, 33, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED, 0, 0},
  {&sensor3, &SENSOR3_WIRE, 0x32, 25, 35, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0}  
};

const int COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

// Motor configuration
int mov_forw = 13; 
int mov_back = 12; 
int enable1Pin = 2;

// Variables to store sensor readings
int left_sensor = 0;
int right_sensor = 0;
int middle_sensor = 0;

// TensorFlow Lite model
tflite::MicroInterpreter* interpreter;
TfLiteTensor* input;
TfLiteTensor* output;

void Initialize_sensors() {
  bool found_any_sensors = false;
  for (int i = 0; i < COUNT_SENSORS; i++)
    digitalWrite(sensors[i].shutdown_pin, LOW);
  delay(10);

  for (int i = 0; i < COUNT_SENSORS; i++) {
    digitalWrite(sensors[i].shutdown_pin, HIGH);
    delay(10);
    if (sensors[i].psensor->begin(sensors[i].id, false, sensors[i].pwire, sensors[i].sensor_config)) {
      found_any_sensors = true;
    } else {
      Serial.print(i, DEC);
      Serial.print(F(": failed to start\n"));
    }
  }
  if (!found_any_sensors) {
    Serial.println("No valid sensors found");
    while (1);
  }
}

void timed_async_read_sensors() {
  uint16_t ranges_mm[COUNT_SENSORS];
  bool timeouts[COUNT_SENSORS];
  uint32_t stop_times[COUNT_SENSORS];

  if(RemoteXY.sensor_button == 1){
    for (int i = 0; i < COUNT_SENSORS; i++) {
      ranges_mm[i] = sensors[i].psensor->startRange();
    }
    for (int i = 0; i < COUNT_SENSORS; i++) {
      ranges_mm[i] = sensors[i].psensor->readRangeResult();
      timeouts[i] = sensors[i].psensor->timeoutOccurred();
      stop_times[i] = millis();
    }
  } else {
    for (int i = 0; i < COUNT_SENSORS; i++) {
      ranges_mm[i] = 0;
    }
  }
  
  left_sensor = ranges_mm[0];
  right_sensor = ranges_mm[1];
  middle_sensor = ranges_mm[2];
}

void sensorTask(void *parameter) {
  while (1) {
    timed_async_read_sensors();   
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void motorServoTask(void *parameter) {
  while (1) {
    RemoteXY_Handler();
    
    // Update RemoteXY variables
    RemoteXY.left_sensor = map(left_sensor, 0, 8000, 0, 100);
    RemoteXY.right_sensor = map(right_sensor, 0, 8000, 0, 100);
    RemoteXY.middle_sensor = map(middle_sensor, 0, 8000, 0, 100);

    if (RemoteXY.connect_flag == 1 && !RemoteXY.emergency_button) {
      // Prepare input for TensorFlow model
      input->data.f[0] = left_sensor / 8000.0f;
      input->data.f[1] = middle_sensor / 8000.0f;
      input->data.f[2] = right_sensor / 8000.0f;

      // Run inference
      TfLiteStatus invoke_status = interpreter->Invoke();
      if (invoke_status != kTfLiteOk) {
        Serial.println("Invoke failed");
        return;
      }

      // Get output from model
      float motor_output = output->data.f[0];
      float servo_output = output->data.f[1];

      // Scale motor output (0 to 100)
      int motor_value = (int)(motor_output * 100);
      motor_value = constrain(motor_value, 0, 100);
      RemoteXY.motor_val = motor_value;

      // Scale servo output (30 left to 30 right)
      int servo_value = (int)(servo_output * 60) - 30;
      servo_value = constrain(servo_value, -30, 30);
      RemoteXY.servo_val = map(servo_value, -30, 30, 0, 100);

      // Control motor
      digitalWrite(enable1Pin, HIGH);
      if (motor_value > 0) {
        analogWrite(mov_forw, motor_value * 2.55);
        digitalWrite(mov_back, LOW);
      } else {
        digitalWrite(mov_forw, LOW);
        analogWrite(mov_back, -motor_value * 2.55);
      }

      // Control servo
      myservo.write(servo_value + 90);
    } else {
      // Emergency stop or disconnected
      digitalWrite(mov_forw, LOW);
      digitalWrite(mov_back, LOW);
      myservo.write(90); // Center position
      RemoteXY.motor_val = 0;
      RemoteXY.servo_val = 50; // Center position
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Starting setup...");
  
  RemoteXY_Init(); 

  Serial.print("Access Point SSID: ");
  Serial.println(REMOTEXY_WIFI_SSID);
  Serial.print("Access Point IP: ");
  Serial.println(WiFi.softAPIP());

  myservo.attach(14);

  pinMode(mov_forw, OUTPUT);
  pinMode(mov_back, OUTPUT);
  pinMode(enable1Pin, OUTPUT); 
  
  Wire.begin();
  
  for (int i = 0; i < COUNT_SENSORS; i++) {
    pinMode(sensors[i].shutdown_pin, OUTPUT);
    digitalWrite(sensors[i].shutdown_pin, LOW);
    if (sensors[i].interrupt_pin >= 0)
      pinMode(sensors[i].interrupt_pin, INPUT_PULLUP);
  }
  
  Initialize_sensors();

  // Initialize TensorFlow Lite model (pseudo-code, replace with actual initialization)
  static tflite::MicroErrorReporter micro_error_reporter;
  static tflite::AllOpsResolver resolver;
  static const tflite::Model* model = tflite::GetModel(g_model);
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize, &micro_error_reporter);
  interpreter = &static_interpreter;

  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    Serial.println("AllocateTensors() failed");
    return;
  }

  input = interpreter->input(0);
  output = interpreter->output(0);

  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(motorServoTask, "MotorServoTask", 4096, NULL, 1, NULL, 1);

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
