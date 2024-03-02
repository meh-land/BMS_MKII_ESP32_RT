/**Libraries*/
#include <ros.h> // ROS

#include "ACS712.h" //acs712
#include "Adafruit_Sensor.h" // necessary for DHT11
#include "DHT.h" //DHT11

/** #TODO: Change #define to enums */

/*********************************VOLTAGE SENSING*********************************/

#define RESISTOR_RATIO  (11.0 / 14.0)

#define VPIN_CELL_01 6  // ADC pin 6
#define VPIN_CELL_02 7  // ADC pin 7
#define VPIN_CELL_03 8  // ADC pin 8

#define NO_OF_CELLS 3
#define CELL_01   0
#define CELL_02   1
#define CELL_03   2

#define CELL_MAX_mV 4200
#define CELL_MIN_mV 3700

float cellVoltageLevels[NO_OF_CELLS] = {0.0}; // frequency array goes brrr! *wink wink* (V levels in V)
unsigned long int cellVoltageTemp = 0; // (V levels in mV)


/*********************************CURRENT SENSING*************************************/

#define ACS_PIN 4 // ADC pin 4

      //ACS(PIN, MAX_VOLTAGE, ADC_STEPS, SENSITIVITY)
ACS712  ACS(A0, 5.0, 1023, 100); // 20A varient has 100mV/A sensitivity
float currentLevel_mA = 0.0;

/*********************************TEMPERATURE SENSING*********************************/

#define TEMP_MAX  45

#define DHT_PIN 5 // ADC pin 5

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

/**************************************BALANCING**************************************/

#define BALANCING_CELL_01   35
#define BALANCING_CELL_02   36
#define BALANCING_CELL_03   37

/***************************************CONTACTORS************************************/

#define CONTACTOR_PIN1  31
#define CONTACTOR_PIN2  33

#define CONTACTOR_SET   23
#define CONTACTOR_RESET 32

bool contactorFlag = false;

void IRAM_ATTR contactorSetConnectionInterrupt() 
{
    digitalWrite(CONTACTOR_PIN1, HIGH);
    digitalWrite(CONTACTOR_PIN2, HIGH);
    contactorFlag = true;
}

void IRAM_ATTR contactorResetConnectionInterrupt() 
{
    digitalWrite(CONTACTOR_PIN1, LOW);
    digitalWrite(CONTACTOR_PIN2, LOW);
    contactorFlag = false;
}

/*********************************FUNCTION PROTOTYPES********************************/
void tempSensor_init();

void currentSensor_init();

void cellVoltageReading_init();

void balancing_init();

void contactor_init();

void interrupt_init();

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
#define READ_VOLTAGE_PRIORITY   19
#define READ_TEMP_PRIORITY      13
#define READ_CURR_PRIORITY      12

// Task 1
void void_RTOSTask_1_ReadCellVoltageLevel(void *parameter)
{
    while(true)
    {
        // read voltage and send it via ROS_serial  
        for (unsigned char local_u8CellNumIterator = CELL_01, local_u8CellPinIterator = VPIN_CELL_01; local_u8CellNumIterator <= CELL_03; local_u8CellNumIterator++, local_u8CellPinIterator++)
        {
                cellVoltageTemp = analogRead(local_u8CellPinIterator) * 3.3 / 1024 * 1000;
                cellVoltageTemp = RESISTOR_RATIO / (NO_OF_CELLS - local_u8CellNumIterator);
      
                // critical section start
                if (cellVoltageTemp > CELL_MAX_mV)
                {
                     // BALANCING GOES BRRR!
                     switch (local_u8CellNumIterator)
                     {
                          case CELL_01: digitalWrite(BALANCING_CELL_01, HIGH); break;
                          case CELL_02: digitalWrite(BALANCING_CELL_02, HIGH); break;
                          case CELL_03: digitalWrite(BALANCING_CELL_03, HIGH); break;
                     }
                      
                }  
                else if (cellVoltageTemp < CELL_MIN_mV)
                {
                     // CHARGING GOES BRRR...?!
                     switch (local_u8CellNumIterator)
                     {
                          case CELL_01: digitalWrite(BALANCING_CELL_01, LOW); break;
                          case CELL_02: digitalWrite(BALANCING_CELL_02, LOW); break;
                          case CELL_03: digitalWrite(BALANCING_CELL_03, LOW); break;
                     } 
                } 
                  
                // critical section end
                
                cellVoltageLevels[local_u8CellNumIterator] = cellVoltageTemp / 1000.0;
                
         }
         // ros_serial => send this topic
         vTaskDelay(500);
    }  
}


// Task 2
void void_RTOSTask_2_ReadTempLevel(void *parameter)
{
    while(true)
    {
        // read temp and send it via ROS_serial  
    
        // Reading temperature or humidity takes about 250 milliseconds!
        //delay(250);
        
        // Read temperature as Celsius (the default)
        tempLevel_C = dht.readTemperature();

        // Read humidity level
        humidityLevel = dht.readHumidity();

        // Compute heat index in Celsius (isFahreheit = false)
        tempIndex = dht.computeHeatIndex(tempLevel_C, humidityLevel, false);

        // critical section start
        if ((tempLevel_C > TEMP_MAX) && !(contactorFlag))
        {
              digitalWrite(CONTACTOR_PIN1, HIGH);
              digitalWrite(CONTACTOR_PIN2, HIGH);
              contactorFlag = true;
        }
        // critical section end

        
        // ros_serial => send this topic
    
        vTaskDelay(1000); // reading is taken once per second
        
    }  
}


// Task 3
void void_RTOSTask_3_ReadCurrentLevel(void *parameter)
{
    while(true)
    {
        // read current and send it via ROS_serial  
    
        currentLevel_mA = ACS.mA_DC() / 1000.0; // return reading in A
    
        // ros_serial => send this topic
    
        vTaskDelay(300);
    
    }  
}


void setup() {
  
  // Serial.begin(9600);
  
  // Temp Setup
  tempSensor_init();

  // ACS712 Setup
  currentSensor_init();

  // Cell Voltage Setup
  cellVoltageReading_init();
  
  // Balancing Pins Setup
  balancing_init();
  
  // Contactor Pins Setup
  contactor_init();

  // Interrupt Setup
  interrupt_init();

  // Creating Tasks
  xTaskCreatePinnedToCore(void_RTOSTask_1_ReadCellVoltageLevel, "Read Cell Voltage Level", 1024, NULL, READ_VOLTAGE_PRIORITY, NULL, pro_cpu);
  xTaskCreatePinnedToCore(void_RTOSTask_2_ReadTempLevel, "Read Temperature Level", 1024, NULL, READ_TEMP_PRIORITY, NULL, app_cpu);
  xTaskCreatePinnedToCore(void_RTOSTask_3_ReadCurrentLevel, "Read Current Level", 1024, NULL, READ_CURR_PRIORITY, NULL, app_cpu);

  // no need to call vTaskStartScheduler() like in vanilla FreeRTOS --- it is called automatically
  
}

void loop() {
  // put your main code here, to run repeatedly:
 
}


void tempSensor_init()
{
  dht.begin();
}

void currentSensor_init()
{
  ACS.autoMidPoint();
}

void cellVoltageReading_init()
{
  pinMode(CELL_01,INPUT);
  pinMode(CELL_02,INPUT);
  pinMode(CELL_03,INPUT);
}

void balancing_init()
{
  pinMode(BALANCING_CELL_01, OUTPUT);
  pinMode(BALANCING_CELL_02, OUTPUT);
  pinMode(BALANCING_CELL_03, OUTPUT);
}

void contactor_init()
{
  pinMode(CONTACTOR_PIN1, OUTPUT);
  pinMode(CONTACTOR_PIN2, OUTPUT);
  pinMode(CONTACTOR_SET, INPUT);
  pinMode(CONTACTOR_RESET, INPUT);
}

void interrupt_init()
{
  pinMode(CONTACTOR_SET, INPUT);
  attachInterrupt(digitalPinToInterrupt(CONTACTOR_SET), contactorSetConnectionInterrupt, FALLING); // active low
  pinMode(CONTACTOR_RESET, INPUT);
  attachInterrupt(digitalPinToInterrupt(CONTACTOR_RESET), contactorResetConnectionInterrupt, FALLING); // active low
}
