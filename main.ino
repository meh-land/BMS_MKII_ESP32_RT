/**Libraries*/
#include <ros.h> // ROS
#include "ACS712.h" //acs712
#include "Adafruit_Sensor.h" // necessary for DHT11
#include "DHT.h" //DHT11
// FreeRTOS is prebuilt in ESP32 Library (Yay!)


/**Defining Pins*/ // #TODO: Change #define to enums
// ADC Pins
#define ACS_PIN 4
#define DHT_PIN 5
#define CELL_01 6
#define CELL_02 7
#define CELL_03 8

#define NO_OF_CELLS 3

// Balancing Pins
#define BALANCING_CELL_01   35
#define BALANCING_CELL_02   36
#define BALANCING_CELL_03   37

// Contactor Pins
#define CONTACTOR_PIN1  31
#define CONTACTOR_PIN2  33


/**Temp*/
// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 3 (on the right) of the sensor to GROUND (if your sensor has 3 pins)
// Connect pin 4 (on the right) of the sensor to GROUND and leave the pin 3 EMPTY (if your sensor has 4 pins)
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor
#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);
float tempLevel_C = 0;
float humidityLevel = 0;
float tempIndex = 0;


/**ACS712*/
      //ACS(PIN, MAX_VOLTAGE, ADC_STEPS, SENSITIVITY)
ACS712  ACS(A0, 5.0, 1023, 100); // 20A varient has 100mV/A sensitivity
unsigned long int currentLevel_mA = 0;

/**Cell Voltage Level*/
float cellVoltageLevels[NO_OF_CELLS + 1] = {0.0}; // frequency array goes brrr! *wink wink*

/**Task Shenanigans*/
/*
 * Algorithm (Preferably with Tasks)
 * Task_1: Read cell voltage levels each 5 seconds + send it with ROS_serial
 * Task_2: Read temperature level each 10 seconds + send it with ROS_serial
 * Task_3: Read current level each 1 second + send it with ROS_serial
 * Interrupt_Setup_1: When current level reaches a certain level, activate CONTACTOR
 * Interrupt_Setup_2: When any cell voltage exceeds max voltage, activate balancing
 * Interrupt_Setup_3: When temperature level reaches a critical level, shut down the whole system
 */


// Restrict ESP32 to only 1 core for now *will change this when everything works as intended*
// xTaskCreatePinnedToCore() -> last argument is app_cpu
// ESP32 has 2 cores: pro && app cpu:
static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;

// Tasks Priorities (0 (lowest) -> 24 (highest))
#define READ_VOLTAGE_PRIORITY   0
#define READ_TEMP_PRIORITY      0
#define READ_CURR_PRIORITY      0

// Task 1
void void_RTOSTask_1_ReadCellVoltageLevel(void *parameter)
{
    while(true)
    {
        // read voltage and send it via ROS_serial  
        for (int i = CELL_01; i <= CELL_03; i++)
        {
            cellVoltageLevels[i] = analogRead(i) * 1024 / 3.3;
            //vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        // ros_serial => send this topic
    }  
}


// Task 2
void void_RTOSTask_2_ReadTempLevel(void *parameter)
{
    while(true)
    {
        // read temp and send it via ROS_serial  
        // Wait a few seconds between measurements.
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        // Reading temperature or humidity takes about 250 milliseconds!
        // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
        float humidityLevel = dht.readHumidity();
        // Read temperature as Celsius (the default)
        float tempLevel_C = dht.readTemperature();
        // Compute heat index in Celsius (isFahreheit = false)
        float tempIndex = dht.computeHeatIndex(tempLevel_C, humidityLevel, false);

        // ros_serial => send this topic
    }  
}


// Task 3
void void_RTOSTask_3_ReadCurrentLevel(void *parameter)
{
    while(true)
    {
        // read current and send it via ROS_serial  
        currentLevel_mA = ACS.mA_DC(); // return reading in mA
        // ros_serial => send this topic
    }  
}


void setup() {
  // Serial.begin(9600);
  
  // Temp Setup
  dht.begin();

  // ACS712 Setup
  ACS.autoMidPoint();

  // Cell Voltage Setup
  pinMode(CELL_01,INPUT);
  pinMode(CELL_02,INPUT);
  pinMode(CELL_03,INPUT);

  xTaskCreate(void_RTOSTask_1_ReadCellVoltageLevel, "Read Cell Voltage Level", 1024, NULL, READ_VOLTAGE_PRIORITY, NULL);
  xTaskCreate(void_RTOSTask_2_ReadTempLevel, "Read Temperature Level", 1024, NULL, READ_TEMP_PRIORITY, NULL);
  xTaskCreate(void_RTOSTask_3_ReadCurrentLevel, "Read Current Level", 1024, NULL, READ_CURR_PRIORITY, NULL);

  // no need to call vTaskStartScheduler() like in Vanilla FreeRTOS --- it is called automatically
  
}

void loop() {
  // put your main code here, to run repeatedly:
 
}
