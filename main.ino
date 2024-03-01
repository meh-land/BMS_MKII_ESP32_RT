// Libraries
#include <ros.h>
//#include <> //FreeRTOS
#include "ACS712.h" //acs712

#include "Adafruit_Sensor.h"
#include "DHT.h" //DHT11




/*
 * Algorithm (Preferably with Tasks)
 * Task_1: Read cell voltage levels each 5 seconds + send it with ROS_serial
 * Task_2: Read temperature level each 10 seconds + send it with ROS_serial
 * Task_3: Read current level each 1 second + send it with ROS_serial
 * Interrupt_Setup_1: When current level reaches a certain level, activate CONTACTOR
 * Interrupt_Setup_2: When any cell voltage exceeds max voltage, activate balancing
 * Interrupt_Setup_3: When temperature level reaches a critical level, shut down the whole system
 */

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


/**ACS712*/
      //ACS(PIN, MAX_VOLTAGE, ADC_STEPS, SENSITIVITY)
ACS712  ACS(A0, 5.0, 1023, 100); // 20A varient has 100mV/A sensitivity


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

  
}

void loop() {
  // put your main code here, to run repeatedly:


  /**Temperature Code*/
  // Wait a few seconds between measurements.
  delay(2000);
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  /**ACS712 Code*/
  int mA = ACS.mA_DC(); // return reading in mA

  /**Cell Voltage Code*/
  float cellVoltage[NO_OF_CELLS + 1] = {0.0};

  for (int i = CELL_01; i <= CELL_03; i++)
  {
      cellVoltage[i] = analogRead(i) * 1024 / 3.3;
  }
  
}
