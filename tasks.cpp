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
        for (unsigned char local_u8CellNumIterator = CELL_01, unsigned char local_u8CellPinIterator = VPIN_CELL_01; local_u8CellNumIterator <= CELL_03; local_u8CellNumIterator++, local_u8CellPinIterator++;)
        {
            cellVoltageTemp = analogRead(local_u8CellPinIterator) * 3.3 / 1024 * 1000;
			cellVoltageTemp = RESISTOR_RATIO / (NO_OF_CELLS - local_u8CellNumIterator);
			
			// critical section start
			if (cellVoltageTemp > CELL_MAX_mV)
				// BALANCING GOES BRRR!
				;
			else if (cellVoltageTemp < CELL_MIN_mV)
				// CHARGING GOES BRRR...?!
				;
			// critical section end
			
			cellVoltageLevels[local_u8CellNumIterator] = cellVoltageTemp / 1000.0;
			
        }
        // ros_serial => send this topic
		//vTaskDelay(5000 / portTICK_PERIOD_MS);
    }  
}


// Task 2
void void_RTOSTask_2_ReadTempLevel(void *parameter)
{
    while(true)
    {
        // read temp and send it via ROS_serial  
		
        // Reading temperature or humidity takes about 250 milliseconds!
        
        // Read temperature as Celsius (the default)
        tempLevel_C = dht.readTemperature();
		
		// ros_serial => send this topic
		
		//vTaskDelay(10000 / portTICK_PERIOD_MS);
		
		
		
		/*// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
        humidityLevel = dht.readHumidity();
        // Compute heat index in Celsius (isFahreheit = false)
        tempIndex = dht.computeHeatIndex(tempLevel_C, humidityLevel, false);
		// Wait a few seconds between measurements.*/
        
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
		
		//vTaskDelay(1000 / portTICK_PERIOD_MS);
		
    }  
}