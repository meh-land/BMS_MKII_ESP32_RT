#ifndef INIT_H_
#define INIT_H_
#pragma once

/** #TODO: Change #define to enums */

/*********************************VOLTAGE SENSING*********************************/

#define RESISTOR_RATIO	(11.0 / 14.0)

#define VPIN_CELL_01 6 	// ADC pin 6
#define VPIN_CELL_02 7	// ADC pin 7
#define VPIN_CELL_03 8	// ADC pin 8

#define NO_OF_CELLS 3
#define CELL_01		0
#define CELL_02		1
#define CELL_03		2

#define CELL_MAX_mV	4200
#define CELL_MIN_mV	3700

float cellVoltageLevels[NO_OF_CELLS] = {0.0}; // frequency array goes brrr! *wink wink* (V levels in V)
unsigned long int cellVoltageTemp = 0; // (V levels in mV)


/*********************************CURRENT SENSING*************************************/

#define ACS_PIN 4 // ADC pin 4

      //ACS(PIN, MAX_VOLTAGE, ADC_STEPS, SENSITIVITY)
ACS712  ACS(A0, 5.0, 1023, 100); // 20A varient has 100mV/A sensitivity
float currentLevel_mA = 0.0;

/*********************************TEMPERATURE SENSING*********************************/

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

/**************************FUNCTION PROTOTYPES**************************/
void tempSensor_init();

void currentSensor_init();

void cellVoltageReading_init();

void balancing_init();

void contactor_init();

#endif