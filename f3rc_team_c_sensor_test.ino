#include <Wire.h>
#include <VL53L0X.h>
#include <Ticker.h>
#include <Adafruit_BNO055.h>


//tickerの書き方のメモですが括弧内で関数、何秒でそれを実行するか、何回繰り返すかを宣言してください
//以下が基本の形です
//Ticker hoge(hogehoge, 0) 関数hogehogeを0usで実行する
void VL53L0X_Get();
//Ticker VL53L0Xticker(VL53L0X_Get, 0); 

//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); //ICSの名前, デフォルトアドレス, 謎
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


// 使用するセンサーの数
#define SENSOR_NUM  4 

//使用するセンサーのリセットのピン
#define SENSOR0 23
#define SENSOR1 19
#define SENSOR2 18
#define SENSOR3 17

#define ADDRESS_DEFAULT 0b0101001 // 0x29
#define ADDRESS_00 (ADDRESS_DEFAULT + 2)
/*
各センサのXSHUTへ接続されているGPIOの配列
*/
const int GPIO_MASK_ARRAY[SENSOR_NUM] = {SENSOR0, SENSOR1, SENSOR2, SENSOR3};
VL53L0X gSensor[SENSOR_NUM]; // 使用するセンサークラス配列



bool VL53L0X_Init(){
  //VL53L0X 初期化
  // まず全てのGPIOをLOW
  for (int i = 0; i < SENSOR_NUM; i++)
  {
    pinMode(GPIO_MASK_ARRAY[i], OUTPUT);
    digitalWrite(GPIO_MASK_ARRAY[i], LOW);
  }

  for (int i = 0; i < SENSOR_NUM; i++) {
    // センサを初期化
    pinMode(GPIO_MASK_ARRAY[i], INPUT);
    if (gSensor[i].init() == true)
    {
      gSensor[i].setTimeout(1000);
      gSensor[i].startContinuous(10);
      int address = ADDRESS_00 + (i * 2);
      gSensor[i].setAddress(address);
    }
    else
    {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" error");
      return false;
    }
  }
  return true;
}


void VL53L0X_Get(){
  //VL53L0X 距離の読み取り
  for (int i = 0; i < SENSOR_NUM; i++) {
    Serial.print(gSensor[i].readRangeSingleMillimeters());
    if (gSensor[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }

    Serial.println();
  }
}


void quat_to_euler(double w, double x, double y, double z){
  double roll, pitch, yaw;
  double ysqr = y * y;

  // roll (x-axis rotation)
  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + ysqr);
  roll = atan2(t0, t1);

  // pitch (y-axis rotation)
  double t2 = +2.0 * (w * y - z * x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  pitch = asin(t2);

  // yaw (z-axis rotation)
  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (ysqr + z * z);  
  yaw = atan2(t3, t4);

  // 180 / PI
  roll *= 57.2957795131;
  pitch *= 57.2957795131;
  yaw *= 57.2957795131;

  Serial.print("roll:");
  Serial.print(roll, 7);
  Serial.print(" pitch:");
  Serial.print(pitch, 7);
  Serial.print(" Yaw:");
  Serial.println(yaw, 7);
}



void setup()
{

  // Wire(Arduino-I2C)の初期化
  Wire.begin();

  if(!bno.begin()) {
    Serial.println("Cannot start BNO055!");
  }
  bno.setExtCrystalUse(false);
  
  // デバッグ用シリアル通信は9600bps
  Serial.begin(9600);

/*
  if(!VL53L0X_Init()){
    Serial.println("VL53L0X initialization failed!");
  }

  VL53L0Xticker.start();
*/
}


void loop()
{

  imu::Quaternion quat = bno.getQuat();
  quat_to_euler(quat.w(), quat.x(), quat.y(), quat.z());

  delay(100);
  //VL53L0Xticker.update();
}
