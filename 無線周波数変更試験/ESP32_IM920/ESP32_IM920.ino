HardwareSerial IM920Serial(0);

// #include <SoftwareSerial.h>
void setup()
{
   
  pinMode(27,OUTPUT);//busy
  digitalWrite(27, HIGH);  
  IM920Serial.begin(19200);
  IM920Serial.print("ECIO\r\n");
  delay(300);
}

void loop()
{
  IM920Serial.print("STCH 31\r\n");
  delay(100);
  while(1){
    IM920Serial.print("TXDA IM920 CH31\r\n");
    delay(800);
   }

  delay(100);
}