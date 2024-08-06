#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_BNO055.h>
#include <moter.h>
#include <motermovement.h>
#include <Ticker.h>

void getlocation();
Ticker getlocationticker(getlocation, 50);

/*
#include <Ticker.h>
tickerの書き方のメモですが括弧内で関数、何秒でそれを実行するか、何回繰り返すかを宣言してください
以下が基本の形です
Ticker hoge(hogehoge, 0) 関数hogehogeを0usで実行する
void VL53L0X_Get();
Ticker VL53L0Xticker(VL53L0X_Get, 0); 
*/

//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); //ICSの名前, デフォルトアドレス, 謎
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


//ここからは距離センサーの設定
#define ADDRESS_DEFAULT 0b0101001 // 0x29
#define ADDRESS_00 (ADDRESS_DEFAULT + 2)

// 使用する距離センサーの数
#define SENSOR_NUM  4 

//使用する距離センサーのリセットのピン
#define SENSOR0 23
#define SENSOR1 19
#define SENSOR2 18
#define SENSOR3 17

const int GPIO_MASK_ARRAY[SENSOR_NUM] = {23, 19, 18, 17};
VL53L0X gSensor[SENSOR_NUM]; // 使用するセンサークラス配列

//モータのピンの設定
#define moter0_0 13
#define moter0_1 12
#define moter0_2 14
#define moter0_3 27
#define moter1_0 26
#define moter1_1 25
#define moter1_2 33
#define moter1_3 32
#define moter2_0 15
#define moter2_1 2
#define moter2_2 0
#define moter2_3 4
#define moter3_0 16
#define moter3_1 17
#define moter3_2 5
#define moter3_3 18

//モーターのclass
Moter moter0, moter1, moter2, moter3;
Motermovement Drive;


//距離センサーから得られた座標を保存しておく場所
typedef struct xy{
  double x1, x2, y1, y2;
} Location;

//9軸センサーから得られた角度を保存しておく場所
//基本的にはクオータニオンを利用して計算するのでほとんど使わない
typedef struct angles{
  double roll, yaw, pitch;
} Angles;


bool vl53l0xInit() {
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
      return(true);
    }
    else
    {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" error");
      return(false);
    }
  }
}

void vl53l0xget() {
  for (int i = 0; i < SENSOR_NUM; i++) {
    Serial.print(gSensor[i].readRangeSingleMillimeters());
    if (gSensor[i].timeoutOccurred()) { 
      Serial.print(" TIMEOUT"); 
    }
    Serial.println();
  }
}


void quat_to_euler(Angles *p, double w, double x, double y, double z) {
  //double roll, pitch, yaw;
  double ysqr = y * y;

  // roll (x-axis rotation)
  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + ysqr);
  p->roll = atan2(t0, t1);

  // pitch (y-axis rotation)
  double t2 = +2.0 * (w * y - z * x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  p->pitch = asin(t2);

  // yaw (z-axis rotation)
  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (ysqr + z * z);  
  p->yaw = atan2(t3, t4);

  /*
  // 180 / PI
  roll *= 57.2957795131;
  pitch *= 57.2957795131;
  yaw *= 57.2957795131;
  */
}


void setup()
{
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバッグ用シリアル通信は9600bps
  Serial.begin(9600);


  if(!bno.begin()) {
    Serial.println("Cannot start BNO055!");
  }

  bno.setExtCrystalUse(false);
  
  
  if(!vl53l0xInit()){
    Serial.println("VL53L0X initialization failed!");
  }




  moter0.set(moter0_0, moter0_1, moter0_2, moter0_3);
  moter1.set(moter1_0, moter1_1, moter1_2, moter1_3);
  moter1.set(moter2_0, moter2_1, moter2_2, moter2_3);
  moter1.set(moter3_0, moter3_1, moter3_2, moter3_3);

  Drive.set(moter0, moter1, moter2, moter3);
}


void loop()
{
  Angles rpy;
  
  imu::Quaternion quat = bno.getQuat();

  //左手系から右手系への変換
  //bno055は左手系らしい、頭おかしい
  double q1 = quat.w();
  double q2 = -quat.x();
  double q3 = quat.y();
  double q4 = -quat.z();

  quat_to_euler(&rpy, q1, q2, q3, q4);

  //回転行列の表現行列
  double a[3][3] = {{q1*q1-q2*q2-q3*q3+q4*q4, 2*(q1*q2+q3*q4), 2*(q1*q3-q2*q4)},
                    {2*(q1*q2-q3*q4), -q1*q1+q2*q2-q3*q3+q4*q4, 2*(q2*q3+q1*q4)},
                    {2*(q1*q3+q2*q4), 2*(q2*q3-q1*q4), -q1*q1-q2*q2+q3*q3+q4*q4}};



  // 加速度センサ値の取得と表示
  imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print(" 　Ac_xyz:");
  Serial.print(accelermetor.x());
  Serial.print(", ");
  Serial.print(accelermetor.y());
  Serial.print(", ");
  Serial.print(accelermetor.z());

  //ここでも左手系から右手系に変換しています死ね
  //rotated_locationは右手系です
  double rotated_accel[3], raw_accel[3] = {accelermetor.y(), accelermetor.x(), accelermetor.z()};
  
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      raw_accel[i] = a[i][j] * raw_accel[j];
    }
  }


  Serial.print(rpy.roll);
  Serial.print(",");
  Serial.print(rpy.pitch);
  Serial.print(",");
  Serial.println(rpy.yaw);
  vl53l0xget();
  delay(100);
}
