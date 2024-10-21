// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ESP32CORE_BLE

#include <BLEDevice.h>

// RemoteXY connection settings 
#define REMOTEXY_BLUETOOTH_NAME "RemoteXY"


#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 194 bytes
  { 255,3,0,59,0,187,0,18,0,0,0,25,2,106,200,200,84,1,1,9,
  0,4,90,59,7,86,178,254,10,87,48,2,82,4,6,81,7,86,11,68,
  92,9,176,2,82,71,37,79,71,71,114,22,56,56,56,3,2,16,135,0,
  0,0,0,0,0,127,67,0,0,72,66,0,0,200,65,0,0,160,64,31,
  0,135,0,0,0,0,0,0,22,67,94,0,0,22,67,0,0,72,67,36,
  0,0,72,67,0,0,127,67,10,239,17,57,57,13,13,21,21,48,134,36,
  31,79,78,0,31,79,70,70,0,67,22,21,21,24,57,26,17,10,68,31,
  26,11,67,36,24,21,24,78,26,16,10,68,31,26,11,67,22,86,21,24,
  50,40,15,9,68,31,26,11,67,33,88,21,24,68,40,15,9,68,31,26,
  11,67,44,88,21,24,86,40,15,9,68,31,26,11 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int8_t motor_slider; // from -100 to 100
  int8_t servo_slider; // from -100 to 100
  uint8_t pushSwitch_01; // =1 if state is ON, else =0

    // output variables
  float speedometer; // from 0 to 255
  char servo_val[11]; // string UTF8 end zero
  char motor_val[11]; // string UTF8 end zero
  char left_sensor[11]; // string UTF8 end zero
  char middle_sensor[11]; // string UTF8 end zero
  char right_sensor[11]; // string UTF8 end zero

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)
 
/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char* serverName = "http://192.168.4.2:8080"; // Replace X.X with your laptop's IP address
unsigned long lastTime = 0;
unsigned long timerDelay = 5000;

// Sensor and Servo Libraries
Servo myservo;

// Define Wire objects for the sensors
#define SENSOR1_WIRE Wire
#define SENSOR2_WIRE Wire
#define SENSOR3_WIRE Wire

// Setup mode for doing reads
typedef enum {
  RUN_MODE_DEFAULT = 1,
  RUN_MODE_ASYNC,
  RUN_MODE_GPIO,
  RUN_MODE_CONT
} runmode_t;

runmode_t run_mode = RUN_MODE_ASYNC;

typedef struct {
  Adafruit_VL53L0X *psensor; // pointer to object
  TwoWire *pwire;
  int id;            // id for the sensor
  int shutdown_pin;  // which pin for shutdown;
  int interrupt_pin; // which pin to use for interrupts.
  Adafruit_VL53L0X::VL53L0X_Sense_config_t sensor_config; // options for how to use the sensor
  uint16_t range;        // range value used in continuous mode stuff.
  uint8_t sensor_status; // status from last ranging in continuous.
} sensorList_t;

// Actual object, could probably include in structure above
Adafruit_VL53L0X sensor1;
Adafruit_VL53L0X sensor2;
Adafruit_VL53L0X sensor3;

// Setup for sensors
sensorList_t sensors[] = {
    {&sensor1, &SENSOR1_WIRE, 0x30, 26, 32, Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE, 0, 0},
    {&sensor2, &SENSOR2_WIRE, 0x31, 27, 33, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED, 0, 0},
    {&sensor3, &SENSOR3_WIRE, 0x32, 25, 35, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0}  
};

const int COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

const uint16_t ALL_SENSORS_PENDING = ((1 << COUNT_SENSORS) - 1);
uint16_t sensors_pending = ALL_SENSORS_PENDING;
uint32_t sensor_last_cycle_time;

// Variables to store sensor readings
int left_sensor = 0;
int right_sensor = 0;
int middle_sensor = 0;

void Initialize_sensors() {
  bool found_any_sensors = false;
  // Set all shutdown pins low to shutdown sensors
  for (int i = 0; i < COUNT_SENSORS; i++)
    digitalWrite(sensors[i].shutdown_pin, LOW);
  delay(10);

  for (int i = 0; i < COUNT_SENSORS; i++) {
    // one by one enable sensors and set their ID
    digitalWrite(sensors[i].shutdown_pin, HIGH);
    delay(10); // give time to wake up.
    if (sensors[i].psensor->begin(sensors[i].id, false, sensors[i].pwire,
                                  sensors[i].sensor_config)) {
      found_any_sensors = true;
    } else {
      Serial.print(i, DEC);
      Serial.print(F(": failed to start\n"));
    }
  }
  if (!found_any_sensors) {
    Serial.println("No valid sensors found");
    while (1)
      ;
  }
}

// Motor A
int mov_forw = 13; 
int mov_back = 12; 
int enable1Pin = 2;

// Setting minimum duty cycle
int dutyCycle = 60;


void timed_async_read_sensors() {
  // First use simple function
  uint16_t ranges_mm[COUNT_SENSORS];
  bool timeouts[COUNT_SENSORS];
  uint32_t stop_times[COUNT_SENSORS];

  if(RemoteXY.pushSwitch_01 == 1){
  // Tell all sensors to start.
  for (int i = 0; i < COUNT_SENSORS; i++) {
    ranges_mm[i] = sensors[i].psensor->startRange();
  }
  // We could call to see if done, but this version the readRange will wait
  // until ready
  for (int i = 0; i < COUNT_SENSORS; i++) {
    ranges_mm[i] = sensors[i].psensor->readRangeResult();
    timeouts[i] = sensors[i].psensor->timeoutOccurred();
    stop_times[i] = millis();
  }
  }
  else{
    for (int i = 0; i < COUNT_SENSORS; i++) {
    ranges_mm[i] = 0;
  }
  }
  // Store sensor values
  left_sensor = ranges_mm[0];
  right_sensor = ranges_mm[1];
  middle_sensor = ranges_mm[2];
}



// Tasks
void sensorTask(void *parameter) {
  while (1) {
    timed_async_read_sensors();   
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void motorServoTask(void *parameter) {
  while (1) {
    RemoteXY_Handler();
    // Store sensor values in RemoteXY variables
    sprintf(RemoteXY.left_sensor, "%d", left_sensor);
    sprintf(RemoteXY.right_sensor, "%d", right_sensor);
    sprintf(RemoteXY.middle_sensor, "%d", middle_sensor);

    // Store motor_slider and servo_slider values in motor_val and servo_val
    sprintf(RemoteXY.motor_val, "%d", RemoteXY.motor_slider);
    sprintf(RemoteXY.servo_val, "%d", RemoteXY.servo_slider);

    if (RemoteXY.connect_flag == 1 && RemoteXY.pushSwitch_01 == 1) {
      int ms = RemoteXY.servo_slider * 0.9 + 90;
      myservo.write(ms);  
  
      int go = RemoteXY.motor_slider;
      RemoteXY.speedometer = go * 2.55;
    
      digitalWrite(enable1Pin, HIGH);

      if (go > 0) {
        analogWrite(mov_forw, go * 2.55);
        digitalWrite(mov_back, LOW);
      } else if (go < 0) {
        digitalWrite(mov_forw, LOW);
        analogWrite(mov_back, -go * 2.55);
      } else {
        digitalWrite(mov_forw, LOW);
        digitalWrite(mov_back, LOW);
      }
    } else {
      digitalWrite(mov_forw, LOW);
      digitalWrite(mov_back, LOW);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS); // Adjust delay as needed
  }
}

void setup() 
{
  Serial.begin(115200);  // Make sure Serial is initialized
  delay(1000);  // Give some time for Serial to be ready
  
  Serial.println("Starting setup...");
  
  RemoteXY_Init (); 

  RemoteXY.speedometer = 0;
  myservo.attach(14);

  RemoteXY.servo_slider = 0;
  RemoteXY.motor_slider = 0;

  pinMode(mov_forw, OUTPUT);
  pinMode(mov_back, OUTPUT);
  pinMode(enable1Pin, OUTPUT); 
  
  Wire.begin();
  
  // initialize all of the pins.
  for (int i = 0; i < COUNT_SENSORS; i++) {
    pinMode(sensors[i].shutdown_pin, OUTPUT);
    digitalWrite(sensors[i].shutdown_pin, LOW);

    if (sensors[i].interrupt_pin >= 0)
      pinMode(sensors[i].interrupt_pin, INPUT_PULLUP);
  }
  
  // Start sensor initialization
  Initialize_sensors();

  // Create tasks
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(motorServoTask, "MotorServoTask", 4096, NULL, 1, NULL, 1);

}

void loop() 
{
  
  // Empty loop as tasks handle the functionality
  vTaskDelay(1000 / portTICK_PERIOD_MS); // Adjust delay as needed
}
