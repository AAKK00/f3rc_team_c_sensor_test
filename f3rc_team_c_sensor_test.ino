#include <Wire.h>
#include <MadgwickAHRS.h>
#include <VL53L0X.h>

Madgwick MadgwickFilter;
#define MadgwickHz 100 //周波数。１秒間に何回データを読み込むかの値

#include <Ticker.h>
//tickerの書き方のメモですが括弧内で関数、何秒でそれを実行するか、何回繰り返すかを宣言してください
//以下が基本の形です
//Ticker hoge(hogehoge, 0) 関数hogehogeを0usで実行する
void BMX055_All();
Ticker madgwickticker(BMX055_All, 0);

void VL53L0X_Get();
Ticker VL53L0Xticker(VL53L0X_Get, 0); 



// BMX055 加速度センサのI2Cアドレス  
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Openの時)
// BMX055 ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)


// センサーの値を保存するグローバル変数
float ax = 0.00;
float ay = 0.00;
float az = 0.00;
float gx = 0.00;
float gy = 0.00;
float gz = 0.00;
int mx = 0.00;
int my = 0.00;
int mz = 0.00;
float pitch = 0.00;
float roll = 0.00;
float yaw = 0.00;


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



bool BMX055_Init() {
  InitAccel();
  InitGyro();
  InitMag();
}

bool InitAccel() {
  WriteByte(Addr_Accl, 0x0F, 0x03); // PMU_Range register, Range = +/- 2g
  WriteByte(Addr_Accl, 0x10, 0x08); // PMU_BW register, Bandwidth = 7.81 Hz
  WriteByte(Addr_Accl, 0x11, 0x00); // PMU_LPW register, Normal mode
}

bool InitGyro() {
  WriteByte(Addr_Gyro, 0x0F, 0x04); // Range register, Full scale = +/- 125 degree/s
  WriteByte(Addr_Gyro, 0x10, 0x07); // Bandwidth register, ODR = 100 Hz
  WriteByte(Addr_Gyro, 0x11, 0x00); // LPM1 register, Normal mode
}

bool InitMag() {
  WriteByte(Addr_Mag, 0x4B, 0x83); // Soft reset
  WriteByte(Addr_Mag, 0x4B, 0x01); // Soft reset
  WriteByte(Addr_Mag, 0x4C, 0x00); // Mag register, Normal Mode, ODR = 10 Hz
  WriteByte(Addr_Mag, 0x4E, 0x84); // Mag register, X, Y, Z-Axis enabled
  WriteByte(Addr_Mag, 0x51, 0x04); // Mag register, No. of Repetitions for X-Y Axis = 9
  WriteByte(Addr_Mag, 0x52, 0x10); // Mag register, No. of Repetitions for Z-Axis = 16
}

bool WriteByte(int address, byte subAddress, byte data) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  if (!Wire.endTransmission()){
    return false;
  }
  return true;
  delay(100); // Short delay after each write
}



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




unsigned int data[8];


//BMX055から全データとMadgwickフィルタの結果を取得する
void BMX055_All() {
  // Read accelerometer data
  readSensorData(Addr_Accl, 0x02, 6);
  ax = convertAccelData(data[1], data[0]);
  ay = convertAccelData(data[3], data[2]);
  az = convertAccelData(data[5], data[4]);

  // Read gyro data
  readSensorData(Addr_Gyro, 0x02, 6);
  gx = convertGyroData(data[1], data[0]);
  gy = convertGyroData(data[3], data[2]);
  gz = convertGyroData(data[5], data[4]);

  readSensorData(Addr_Mag, 0x42, 8);
  // Convert the data
  mx = ((data[1] << 5) | (data[0] >> 3));
  if (mx > 4095)  mx -= 8192;
  my = ((data[3] << 5) | (data[2] >> 3));
  if (my > 4095)  my -= 8192;
  mz = ((data[5] << 7) | (data[4] >> 1));
  if (mz > 16383)  mz -= 32768;

  MadgwickFilter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  float pitch = MadgwickFilter.getPitch();
  float roll  = MadgwickFilter.getRoll();
  float yaw   = MadgwickFilter.getYaw();
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.print(yaw);
  Serial.println("");
}

void readSensorData(int address, byte startReg, int numBytes) {
  Wire.beginTransmission(address);
  Wire.write(startReg); // Select start register
  Wire.endTransmission();
  
  Wire.requestFrom(address, numBytes);
  for (int i = 0; i < numBytes; i++) {
    if (Wire.available()) {
      data[i] = Wire.read(); // Read data bytes
    }
  }
}

float convertAccelData(byte msb, byte lsb) {
  int raw = (msb << 8) | lsb;
  if (raw > 2047) raw -= 4096; // Convert to signed 12-bit
  return raw * 0.00098; // Convert to gravity units (+/-2g)
}

float convertGyroData(byte msb, byte lsb) {
  int raw = (msb << 8) | lsb;
  if (raw > 32767) raw -= 65536; // Convert to signed 16-bit
  return raw * 0.0038; // Convert to degrees per second (+/-125°/s)
}



void VL53L0X_Get(){
  //VL53L0X 距離の読み取り
  for (int i = 0; i < SENSOR_NUM; i++) {
    Serial.print(gSensor[i].readRangeSingleMillimeters());
    if (gSensor[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }

    Serial.println();
  }
}


void setup()
{
  MadgwickFilter.begin(MadgwickHz);

  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバッグ用シリアル通信は115200bps
  Serial.begin(115200);


  //BMX055 初期化
  if(!BMX055_Init()){
    Serial.println("BMX055 initialization failed!");
  }

  if(!VL53L0X_Init()){
    Serial.println("VL53L0X initialization failed!");
  }

  VL53L0Xticker.start();
  madgwickticker.start();
}


void loop()
{
  VL53L0Xticker.update();
  madgwickticker.update();
}
