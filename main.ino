/**Libraries*/
#include <ros.h> // ROS
#include "ACS712.h" //acs712
#include "Adafruit_Sensor.h" // necessary for DHT11
#include "DHT.h" //DHT11
#include "init.h"
#include "tasks.cpp"
// FreeRTOS is prebuilt in ESP32 Library (Yay!)


/**Defining Pins*/ // #TODO: Change #define to enums
// ADC Pins








// Balancing Pins


// Contactor Pins



/**Temp*/



/**ACS712*/


/**Cell Voltage Level*/





void setup() {
  // Serial.begin(9600);
  
  // Temp Setup
  

  // ACS712 Setup
  

  // Cell Voltage Setup
  
  
  // Balancing Pins Setup
  
  
  // Contactor Pins Setup
  

  xTaskCreate(void_RTOSTask_1_ReadCellVoltageLevel, "Read Cell Voltage Level", 1024, NULL, READ_VOLTAGE_PRIORITY, NULL);
  xTaskCreate(void_RTOSTask_2_ReadTempLevel, "Read Temperature Level", 1024, NULL, READ_TEMP_PRIORITY, NULL);
  xTaskCreate(void_RTOSTask_3_ReadCurrentLevel, "Read Current Level", 1024, NULL, READ_CURR_PRIORITY, NULL);

  // no need to call vTaskStartScheduler() like in Vanilla FreeRTOS --- it is called automatically
  
}

void loop() {
  // put your main code here, to run repeatedly:
 
}
