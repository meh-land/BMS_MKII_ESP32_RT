
float temp = 0.0;
float v_v = 0.0;
float slope = 140/11;
float constant = 95/11;
#define DUCK 4
#define DUCKING_CONST 3

void setup() {
  // put your setup code here, to run once:
  analogReadResolution(12);
  Serial.begin(115200);
  pinMode(DUCK, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i < 10000; i++)
    v_v += (analogRead(DUCK) * 3.3 / 4096.0);
  v_v /= 10000;
  Serial.println(v_v);
  temp = (v_v * slope) + constant + DUCKING_CONST;
  Serial.println(temp);
  delay(250);
}
