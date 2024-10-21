#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();


void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit VL53L0X test");
  if(!lox.begin()){
    Serial.println(F("Failed to boot sensor"));
  }
  Serial.println(F("VL53L0X Simple Ranging example"));
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  Serial.println("Reading a measurement...");
  lox.rangingTest(&measure,false);
  if(measure.RangeStatus!=4){
    Serial.println("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  }
  else{
    Serial.println("out of range");
  }
  delay(100);

}
