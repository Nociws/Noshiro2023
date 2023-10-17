#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Ticker.h>
#include <TinyGPSPlus.h>
#include <Adafruit_DPS310.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <INA226_asukiaaa.h>
// Ticker bno055ticker; //タイマー割り込み用のインスタンス。
// #define BNO055interval 50 //何ms間隔でデータを取得するか
Ticker GPSticker;
#define GPSinterval 50
Ticker Timeticker;
#define Timeinterval 20000
Ticker dataticker;
#define datainterval 500
Ticker stopticker;
#define stopinterval 12000
Ticker IMticker;
#define IMinterval 100

unsigned long previousMillis = 0;
int interval = 16000;  // インターバルをミリ秒単位で設定

float  L_speed ;
float  R_speed ;

int gp = 0;
int apo = 0;
int cari = 0;
int KEY = 0;
int i = 0;
int k = 0;
int yea = 0;
int g = 0;


//HCSR04
#define echoPin 12 // Echo Pin
#define trigPin 13 // Trigger Pin
double Duration = 0; //受信した間隔
double Distance = 0; //距離
double Distance2 =100; //冗長系
//

//高度検知
float H1 = 230.0; //ここの値使われてない、気圧と温度からきてる
float H2 = 130.0;
float H3 = 60.0;
float H4 = 30.0;
int count = 0;
//

//INA228
int INA228_ADR = 0b01000101; //INA228のアドレス設定
//int INA228_ADL = 0b1000000x; //INA228のアドレス設定
int16_t ma;
float val; 
 String dataString;
 String datapower;

const uint16_t ina226calib = INA226_asukiaaa::calcCalibByResistorMilliOhm(2); // Max 5120 milli ohm
// const uint16_t ina226calib = INA226_asukiaaa::calcCalibByResistorMicroOhm(2000);
INA226_asukiaaa voltCurrMeter(INA226_ASUKIAAA_ADDR_A0_VDD_A1_VDD, ina226calib);


//

//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); //ICSの名前, デフォルトアドレス, 謎
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
int C_mag = 0;
//
//DPS関係S
Adafruit_DPS310 dps;
// DPS310のI2Cアドレス 任意に指定
#define Addr_DPS310 0x55
float P0;   //初期気圧
float P;
float T;
float H;
unsigned long now = 0;
//
//GPS関係
TinyGPSPlus gps;
HardwareSerial ss(2);
static const uint32_t GPSBaud = 9600;
float y = 0; //緯度方向の距離
float x = 0; //経度方向の距離
float dis_1 = 10; //現在地から目標地点までの距離
float dis_2 = 5;
float dis_3 = 9999999999999;
float Lat = 0;
float Lng = 0;
float alt = 0;
float arctan;
float Az;//舵角
float  euler_x; 
float  euler_y;
float  euler_z;
//モタドラ
int left_direc = 14;
int right_direc = 33;
int left = 27;
int right = 32;
//モタドラ

void setup()
{
  pinMode( echoPin, INPUT );
  pinMode( trigPin, OUTPUT );
  pinMode(21, INPUT_PULLUP); //SDA 21番ピンのプルアップ(念のため)
  pinMode(22, INPUT_PULLUP); //SDA 22番ピンのプルアップ(念のため)
  pinMode(33,OUTPUT);
  pinMode(14,OUTPUT);
  pinMode(27,OUTPUT);
  pinMode(32,OUTPUT);
  pinMode(4,OUTPUT);  
  Wire1.begin(); //I2Cをマスターとして動作させる
  Serial.begin(9600);
  Serial1.begin(19200, SERIAL_8N1, 0, 2);//IM920sL
  Serial1.print("TXDA ECIO\r\n");
  ss.begin(GPSBaud, SERIAL_8N1, 16, 17);//GPS
  pinMode(21, INPUT_PULLUP); //SDA 21番ピンのプルアップ(念のため)
  pinMode(22, INPUT_PULLUP); //SDA 22番ピンのプルアップ(念のため)
  if (! dps.begin_I2C()) {          
   Serial.println("Failed to find DPS310");
   Serial1.print("TXDA Failed to find DPS310\r\n");
   while (1) yield();}
  Serial.println("DPS310 OK!");
  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
//INA228
    if (voltCurrMeter.begin() != 0) {
    Serial.println("Failed to begin INA226");
    Serial1.print("TXDA Failed to begin INA228\r\n");
  }
//
    if(!SD.begin()){
        Serial.println("Card Mount Failed");
        Serial1.print("TXDA SD Card Mount Failed\r\n");
        while (1);
    }
  
  writeFile(SD, "/1.txt",dataString);
  writeFile(SD, "/2.txt",datapower);

  if (!bno.begin()) // センサの初期化
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    Serial1.print("TXDA Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!\r\n");
    while (1);
  }
  Serial.print("let's go");
  Serial1.print("TXDA let's go\r\n");
  //Serial.print("$PMTK251,19200*22\r\n");
  Serial.print("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");
  delay(50);
  //Serial.print("$PMTK220,200*2C\r\n");
  delay(50);
  bno.setExtCrystalUse(false);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  //bno055ticker.attach_ms(BNO055interval, get_bno055_data);
  dataticker.attach_ms(datainterval, data);
  Timeticker.attach_ms(Timeinterval, Time);
  //stopticker.attach_ms(stopinterval, stop_gps);
  //IMticker.attach_ms(IMinterval, IM);
  GPSticker.attach_ms(GPSinterval, GPS);
  sensors_event_t temp_event, pressure_event;
  // while (!dps.temperatureAvailable() || !dps.pressureAvailable()) {
  //  return; // wait until there's something to read
  // }
  dps.getEvents(&temp_event, &pressure_event);
  P0 = pressure_event.pressure*1000;
  //  Serial.print(F("Pressure = "));
  Serial.print("Pressure = ");
  Serial.print(P0);
  Serial.println(" Pa");   
}

void get_bno055_data()
{
  Wire1.beginTransmission(0x28);   
  // キャリブレーションのステータスの取得と表示
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  //Serial.print(", Mg");
  Serial.println(C_mag = mag, DEC);
  delay(50);
  Wire1.endTransmission();

  // センサフュージョンによる方向推定値の取得と表示
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  euler_x = euler.x();
  euler_y = euler.y();
  euler_z = euler.z();
  
}

void BNO055(){
  Wire1.beginTransmission(0x28); 
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  euler_x = euler.x();
  Wire1.endTransmission();
}

void GPS(){
//GPS
  if(ss.available() > 0){
       if(i < 1){ 
  //Serial.print("$PMTK251,19200*22\r\n"); 
  Serial.write(ss.read());
  Serial.print("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");
  Serial.write(ss.read());
   delay(100);
   i += 1;
  }        
  }  
  //Serial1.println("TXDA gps\r\n");
    char c = ss.read();
    gps.encode(c);
    if (gps.location.isUpdated()){
  Lat = gps.location.lat();
  Lng = gps.location.lng();
  }

  // }
  // else{
  //   Serial1.println("TXDA wait\r\n");
  // }
  //GPS
}

void writeFile(fs::FS &fs, const char * path, String  message){
    // Serial.print("Writing file:");
    // Serial.print("");
    // Serial.println(path);
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        // Serial.println("Failed to open file for writing");
        return;
    }
    file.print(message);
    // if(file.print(message)){
    //     Serial.println("File written");
    // } else {
    //     Serial.println("Write failed");
    // }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, String  message){
    // Serial.print("Appending to file:");
    // Serial.print("");
    // Serial.print(path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        // Serial.println("Failed to open file for appending");
        return;
    }
    file.print(message);
    // if(file.print(message)){
    //     Serial.println("Message appended");
    // } else {
    //     Serial.println("Append failed");
    // }
    file.close();
}

// void power(){
//   voltCurrMeter.readMA(&ma);
// }

void DPS310(){
    //DPS310
   
  sensors_event_t temp_event, pressure_event;
  while (!dps.temperatureAvailable() || !dps.pressureAvailable()) {
    return; // wait until there's something to read
  }
  dps.getEvents(&temp_event, &pressure_event);
  
//Serial.print("P0");Serial.println(""); Serial.println(P0);
 Wire1.beginTransmission(0x55);  
 P = pressure_event.pressure*1000;
 T = temp_event.temperature;
 H = ((pow(( P0 /P),0.19022256)-1.0)*(T+273.15))/0.0065; //よくわからんけど(T-273.15)になってたから＋にした
 //Serial.print(P); Serial.print(""); Serial.println("Pa");
 //Serial.print(T); Serial.print(""); Serial.println("℃");
//  Serial.print(H); Serial.print(""); Serial.println("m");
 //now = millis();
//  Serial.print(now); Serial.println("ms");
 //hight = (273.15+  float(temp_event.temperature))*fmod((pow(fmod(985.38, float(pressure_event.pressure) ),0.1902)-1),0.0065);
Wire1.endTransmission();
}


void distance(){
  //alt = gps.altitude.meters();
  y = (40.142665 - Lat)*111319.49; //GLatに目標地点の緯度を代入//グラウンド43.824388,143.902931//倉庫43.824226,143.904061
  x = (139.987621 - Lng)*cos(Lat*PI*180)*111319.49;                                //GLngに目標地点の経度を代入
  arctan = atan2(x,y) * 180.0 / PI;   //機体から固定座標への方位計算
  dis_1 = sqrt(pow(y,2) + pow(x,2));       
  dis_3 = dis_2 ;
  dis_2 = dis_1 ;   
  
  // if(apo == 250){
  //   stop();
  //   Lat = gps.location.lat();
  //   Lng = gps.location.lng();
  //   delay(7000);    
  //   apo = 0;

  // }
}

void angle(){
  //方位推定
  euler_x = (euler_x - 90);
  // Serial.print("euler_x:");
  // Serial.print(euler_x);
  // if (arctan < 0){
    
  //   arctan = (360 + arctan) ;
  // }
  
    Az = (euler_x - (arctan+9));
if(Az > 360){
Az = Az -360;  
}     

     //北見の磁北修正角度9.00
  // Serial.println("arctan");
  // Serial.print(arctan);
  // Serial.print("euler_x");
  // Serial.println(euler_x);
  // Serial.print("Az:");
  // Serial.println(Az);

  //Serial.println((String)"目的地への方位Az:"+Az+(String)"°");
  }

void AzimuthAjust(){
//方位修正
  if(Az >360){
    Az = Az -360;
  }
  
  if(Az > 20 || Az < -20){
      while (Az > 20 || Az < -20){
      Serial1.print("TXDA "+String(Az)+"\r\n");
      Serial1.print("TXDA 方位修正\r\n");
      //L_curve();
      if(dis_1 <= 5){
        L_speed = 83;R_speed = 83;
      }
      L_speed = 85;R_speed = 85;

      L_rotation();

      power_check();
      stack2();

      GPS();
      distance();
      BNO055();
      angle();
      delay(50);
    }
    
    if ( Az < -20) {  //左回転/////試運転後角度調整する
      while (Az > 20 || Az < -20){
      // Serial.print(Az);    
      Serial1.println("TXDA 左回転！\r\n");
      Serial1.print("TXDA "+String(Az)+"\r\n");

      L_curve();
      BNO055();
      angle();
      delay(100);
      }
    }
  }
}

void hcsr04(){
  yea += 1;
  if(yea >= 166){
              stop();
          Serial1.print("TXDA GOAL!!\r\n");
          delay(5000000);
  }

  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2); 
  digitalWrite( trigPin, HIGH ); //超音波を出力
  delayMicroseconds( 10 ); //
  digitalWrite( trigPin, LOW );
  
  Duration = pulseIn( echoPin, HIGH ); //センサからの入力
  delay(50);
  if (Duration > 0) {
    Duration = Duration/2; //往復距離を半分にする
    Distance = Duration*340*100/1000000; // 音速を340m/sに設定
    if(Distance <= 30){
      if(Distance2 <= 4){
        if(Distance <= 4){
          stop();
          Serial1.print("TXDA GOAL!!\r\n");
          delay(5000000);
        }
        
    }
    Distance2 = Distance ;

    }
  }
}

void L_rotation(){
  digitalWrite(left_direc,HIGH);
  digitalWrite(right_direc,LOW);
  // L_speed = 93;R_speed = 93;
  analogWrite(left, L_speed);
  analogWrite(right, R_speed);
  //delay(30);  
  // L_speed = 90;R_speed = 90;//delay(30); 
  // //L_speed = 105;R_speed = 105;
  // // 得たアナログ値を1/4して、0-1023の値を0-255に変換
  // analogWrite(left, L_speed);
  // analogWrite(right, R_speed);
    //delay(1000);
  // change_s();
  // power();
  // Serial.println(dataString);
  // appendFile(SD, "/power_test.txt",dataString);
}

void R_rotation(){
  digitalWrite(left_direc,LOW);
  digitalWrite(right_direc,HIGH);
  L_speed = 35;R_speed = 35;
  analogWrite(left, L_speed);
  analogWrite(right, R_speed);
  delay(80);  
  L_speed = 75;R_speed = 75;//delay(30); 
  //L_speed = 105;R_speed = 105;
  // 得たアナログ値を1/4して、0-1023の値を0-255に変換
  analogWrite(left, L_speed);
  analogWrite(right, R_speed);

}


void straight(){
  digitalWrite(left_direc,LOW);
  digitalWrite(right_direc,LOW);
  // L_speed = 100;R_speed = 100;
  // delay(50);  
  // analogWrite(left, L_speed);
  // analogWrite(right, R_speed); 
  L_speed = 145;R_speed = 145;
  // 得たアナログ値を1/4して、0-1023の値を0-255に変換
  analogWrite(left, L_speed);
  analogWrite(right, R_speed);

}

void back(){
  digitalWrite(left_direc,HIGH);
  digitalWrite(right_direc,HIGH);
    L_speed = 140;
    R_speed = 140;
  // 得たアナログ値を1/4して、0-1023の値を0-255に変換
  analogWrite(left, L_speed);
  analogWrite(right, R_speed);

}

void stop(){
  digitalWrite(left_direc,HIGH);
  digitalWrite(right_direc,HIGH);
    L_speed = 0;
    R_speed = 0;
  // 得たアナログ値を1/4して、0-1023の値を0-255に変換
  analogWrite(left, L_speed);
  analogWrite(right, R_speed);

}

void stop_gps(){
  digitalWrite(left_direc,HIGH);
  digitalWrite(right_direc,HIGH);
    L_speed = 0;
    R_speed = 0;
  // 得たアナログ値を1/4して、0-1023の値を0-255に変換
  analogWrite(left, L_speed);
  analogWrite(right, R_speed);
  delay(4500);

}

void goal(){
  while(dis_1 <= 5){
  interval = 8000;  
  digitalWrite(left_direc,LOW);
  digitalWrite(right_direc,LOW);
    L_speed = 117;
    R_speed = 117;
  analogWrite(left, L_speed);
  analogWrite(right, R_speed);
  distance();
  BNO055();
  angle();
  AzimuthAjust();

  delay(100);  
  Serial1.print("TXDA 0m誘導開始\r\n"); 
  hcsr04();
  }
  interval = 15000;
}

// void goal(){
//   if(dis_1 <= 6){
//   while(dis_3 < dis_1){
//   digitalWrite(left_direc,LOW);
//   digitalWrite(right_direc,LOW);
//     L_speed = 82;
//     R_speed = 82;
//   analogWrite(left, L_speed);
//   analogWrite(right, R_speed);
//   if(apo = 10){
//     break;
//   }
//   apo += 1;
//   delay(1000);
//   }
//     // if(dis_1 == 0){
//     Serial1.print("TXDA 超音波測定開始\r\n"); 
//     hcsr04();
//     // }
//   }
// }

void eight(){
  digitalWrite(left_direc,LOW);
  digitalWrite(right_direc,LOW);
  // 得たアナログ値を1/4して、0-1023の値を0-255に変換
  analogWrite(left, L_speed);
  analogWrite(right, R_speed);
    L_speed = 80;R_speed = 80;delay(10);  
  L_speed = 80;R_speed = 90;delay(10); 
  L_speed = 80;R_speed = 100;
  delay(1500);
  analogWrite(left, L_speed);
  analogWrite(right, R_speed);
    L_speed = 80;R_speed = 80;delay(10);  
  L_speed = 90;R_speed = 80;delay(10); 
  L_speed = 100;R_speed = 80;
  delay(1500);
  stop();
}

void L_curve(){
  digitalWrite(left_direc,LOW);
  digitalWrite(right_direc,LOW);
    L_speed = 95;R_speed = 95;
  delay(50);  
    L_speed = 95;
    R_speed = 180;
  // 得たアナログ値を1/4して、0-1023の値を0-255に変換
  analogWrite(left, L_speed);
  analogWrite(right, R_speed);
}

void R_curve(){
  digitalWrite(left_direc,LOW);
  digitalWrite(right_direc,LOW);
    L_speed = 170;
    R_speed = 175;
  // 得たアナログ値を1/4して、0-1023の値を0-255に変換
  analogWrite(left, L_speed);
  analogWrite(right, R_speed);
}

void R_back(){
  digitalWrite(left_direc,HIGH);
  digitalWrite(right_direc,HIGH);
    L_speed = 150;
    R_speed = 175;
  // 得たアナログ値を1/4して、0-1023の値を0-255に変換
  analogWrite(left, L_speed);
  analogWrite(right, R_speed);
}

void power_check(){
  Wire1.beginTransmission(0x40);
  Wire1.write(1000000);//0x07
  Wire1.endTransmission();
  Wire1.requestFrom(0x40,3);
  //val = Wire1.read();
  voltCurrMeter.readMA(&ma);
}

void stack(){
  if(ma >= 800 || ma <= -800){
  R_back();
  delay(1500);
  R_curve();
  delay(3000);
  }
}

void stack2(){
  if(ma >= 650 || ma <= -650){
  R_back();
  delay(1500);
  R_curve();
  delay(3000);
  }
}

void landing(){
//上昇検知
  while (count < 2) {
    // ここに条件を記述します
    if(k == 2){
      break;
    }
    Serial1.print("TXDA 上昇検知１"+String(count)+"\r\n");
    DPS310();
    H1 = H;
    delay(2000);
    Serial1.print("TXDA 上昇検知２"+String(count)+"\r\n");
    DPS310();
    H2 = H;
    H3 = H2-H1;
    Serial1.print("TXDA "+String(H3)+"\r\n");
    if (H3 > 2.0) {
      count++; // 条件がクリアした回数をカウントアップ
    } else {
      count = 0; // 条件がクリアしていない場合はカウントをリセット
    }
  }

  count = 0;
  Serial1.print("TXDA 上昇検知!!!\r\n");
  delay(1000);
  
  while (count < 2) {
    // ここに条件を記述します
    if( k == 2){
break;      
    }
    Serial1.print("TXDA 下降検知１"+String(count)+"\r\n");
    DPS310();
    H1 = H;
    delay(2750);
    Serial1.print("TXDA 下降検知２"+String(count)+"\r\n");
    DPS310();
    H2 = H;
    H3 = H1-H2;
    Serial1.print("TXDA "+String(H3)+"\r\n");
    if (H3 > 1.0) {
      count++; // 条件がクリアした回数をカウントアップ
    } else {
      count = 0; // 条件がクリアしていない場合はカウントをリセット
    }
  }

  count = 0;
  Serial1.print("TXDA 下降検知!!!\r\n");
  delay(1000);

  while (count < 3) {
    // ここに条件を記述します
        
    if( k == 2){
break;      
    }
    Serial1.print("TXDA 着地検知1"+String(count)+"\r\n");
    DPS310();
    H1 = H;
    delay(2000);
    Serial1.print("TXDA 着地検知2"+String(count)+"\r\n");
    DPS310();
    H2 = H;
    H3 = H1-H2;
    Serial1.print("TXDA "+String(H3)+"\r\n");
    if (H1-H2 < 0.5) {
      count++; // 条件がクリアした回数をカウントアップ
    } else {
      count = 0; // 条件がクリアしていない場合はカウントをリセット
    }
  }
    Serial1.print("TXDA 着地検知!!!!\r\n");
    //delay(100); // 適切なディレイを追加してください
    digitalWrite(4,HIGH);  //  
    delay(5000);    
    digitalWrite(4,LOW);


}

void resety(){
  KEY == Serial.read();
  if(7 ==KEY){
    count = 0;
    cari = 0;
}
}

void data(){

  datapower = "";
  datapower += String(ma);
  datapower += ",";
  datapower += String(Distance);
  datapower += "\n";
  appendFile(SD, "/2.txt",datapower);  

  distance();
  delay(20);
  BNO055(); 
  angle();
  dataString = "";
  dataString += String(H);
  dataString += ",";
  dataString += String(arctan);
  dataString += ",";
  dataString += String(euler_x);
  dataString += ",";
  dataString += String(Az);
  dataString += ",";
  dataString += String(dis_1);
  dataString += "\n";
  appendFile(SD, "/1.txt",dataString);




}

// void IM(){
//   Serial1.print("TXDA"+String(H)+","+String(arctan)+","+String(euler_x)+","+String(Az)+","+String(dis_1)+","+String(ma)+","+String(Distance)+"\r\n");
// }
void Time(){
  if(millis() > 15000 && k == 0){
  digitalWrite(4,HIGH);  //  
  delay(5000);    
  digitalWrite(4,LOW);
  k = 2;
  }
}

void loop()
{
  //resety();
  //Serial.print("a");

  if(k == 0){
  landing();
  k = 1; 
  delay(1000);
  straight();//
  delay(8000);
  stop();  
  }

  if(k = 2){
    Serial1.print("TXDA タイマー溶断作動\r\n");    
  }
  if(k = 2){
  while(i < 2){
  while(C_mag < 1){
    L_curve();    
    get_bno055_data();
    Serial1.print("TXDA"+String(C_mag)+"\r\n");        
    delay(100);
  }
  //   while(C_mag < 2){
  //   L_curve();
  //   delay(1000);    
  //   R_rotation();
  //   stop();
  //   get_bno055_data();
  //   Serial1.print("TXDA"+String(C_mag)+"\r\n");        
  //   delay(100);
    
  // }
  //stop();
  // while(C_mag < 3){
  // eight();
  // get_bno055_data();
  // Serial1.print("TXDA"+String(C_mag)+"\r\n");   
  // stop();
  // delay(100);
  // }  
  i += 3;
  }
  }
  Serial1.print("TXDA スタート\r\n");   
  delay(100);
  

  // while(cari < 1){
  //   L_rotation();    
  //   get_bno055_data();
  //   Serial1.print("TXDA"+String(C_mag)+"\r\n");        
  //   delay(100);
  //   cari += 1;
  // }

  while(Lat == 0){
  distance();
  Serial1.print("TXDA gps wait\r\n");
  delay(50);
  }
  
  //DPS310();
  // distance();
  // delay(20);
  // BNO055(); 
  // angle();
  AzimuthAjust();//方位修正  
  //Serial1.print("TXDA 方位修正\r\n");
  Serial1.print("TXDA"+String(arctan)+","+String(euler_x)+","+String(Az)+","+String(dis_1)+"\r\n");
  //Serial1.print("TXDA"+String(arctan)+","+String(euler_x)+","+String(Az)+","+String(dis_1)+","+String(ma)+","+String(Distance)+"\r\n");
  straight();
  Serial1.print("TXDA straight\r\n");
  //Serial1.print("TXDA power_check\r\n");
  power_check();//多分ここがsetupにもどる原因
  //delay(30);
  // Serial1.print("TXDA stack\r\n");
  stack();
  //delay(30);
  goal();
  unsigned long currentMillis = millis();  // 現在の時間を取得
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // 前回の時間を更新
  stop_gps();
  }
  // delay(20);
    
  // apo += 1;
}