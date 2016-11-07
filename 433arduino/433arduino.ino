#define RF_DATA_PIN A1


void setup() {               
  pinMode(RF_DATA_PIN, INPUT_PULLUP);
  pinMode(13, OUTPUT);
//  Serial.begin(921600);
  Serial.begin(115200);
}


/*void loop()
{
  
}*/
void loop()
{
  unsigned long lastTime = micros();
  unsigned long nowTime = micros();
  int a = analogRead(RF_DATA_PIN);
  unsigned int level = a > 400 ? 1 : 0;
 
  while(1) {
    nowTime = micros();
    a = analogRead(RF_DATA_PIN);
    char new_level = a > 400 ? 1 : 0;
    if(new_level != level) {
      Serial.print(nowTime - lastTime);Serial.print("\t");
      Serial.println(level);
      lastTime = nowTime; 
      level = new_level;
    }
  }
}
