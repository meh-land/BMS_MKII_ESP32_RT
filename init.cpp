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
}