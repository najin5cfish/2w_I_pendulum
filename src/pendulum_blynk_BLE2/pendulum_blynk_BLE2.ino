/*
@brief blynkを用いたPID倒立振子
@author S.Kawana
@date 2021.02.21
*/

//bylnk用ライブラリ
#define BLYNK_PRINT Serial
#define BLYNK_USE_DIRECT_CONNECT
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>

//その他ライブラリ
#include "MPU6886.h"
#include <Kalman.h>
#include "FastLED.h"


#define rote_pin_A 33
#define rote_pin_B 19
#define PWM_pin_A 23
#define PWM_pin_B 22
#define button 39
#define LED_pin 27
#define NUM_LEDS 25

//自分の環境に合わせて変更(blynkで取得)
char auth[] = "EtQUJA0Kxhk3lBfdGUn7mLz2lDOgAvRH";
MPU6886 IMU;

//制御時間
unsigned long oldTime = 0, loopTime, nowTime;
float dt;


//IMUの設定
float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float accXoffset = 0, accYoffset = 0, accZoffset = 0;
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;

//accY,accZとgyroxよりx軸の回転角(roll)を使う.
Kalman kalmanX;
float kalAngleX;

//PD制御の設定
byte countS = 0;
int OmegaI = 0;
long AngleI = 0;
int target_Omega = 0;//追加
long target_Angle = 0;//追加
int recOmegaI[10];
int power;
long sumPower = 0;//sumPowor
long sumSumP = 0;//sumSump
int kAngle = 0;
int kOmega = 0;
long kSpeed = 0;
long kDistance = 0;
long vE5 = 0;
long xE5 = 0;

//操縦用
int rightpower;
int leftpower;

// =================
// functions -------
// =================
//センサオフセット算出
void offset_cal() {
  for (int i = 0; i < 100; i++) {
    IMU.getAccelData(&accX, &accY, &accZ);
    IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
    //Serial.print(" accZ : ");
    //Serial.println(accZ);
    delay(10);
    accXoffset += accX;
    accYoffset += accY;
    accZoffset += accZ;
    gyroXoffset += gyroXoffset;
    gyroYoffset += gyroYoffset;
    gyroZoffset += gyroZoffset;
  }
  accXoffset /= 100;
  accYoffset = accYoffset / 100 + 1.0;
  accZoffset /= 100;
  gyroXoffset /= 100;
}

//加速度データより傾き[deg]を取得
float get_theta_acc() {
  float theta_acc = 0;
  IMU.getAccelData(&accX, &accY, &accZ);
  //rad→deg
  theta_acc = atan( -1.0 * (accZ - accZoffset) / (accY - accYoffset) ) * 180.0 / M_PI;
  return theta_acc;
}

//x軸　角速度取得
float get_theta_dot() {
  float theta_dot = 0;
  IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  theta_dot = gyroX - gyroXoffset;
  return theta_dot;
}

// =================
// main start ------
// =================
void setup() {
  Serial.begin(115200);
  Blynk.setDeviceName("Blynk");
  Blynk.begin(auth);
  Serial.println("connected");

  pinMode(button, INPUT);
  pinMode(rote_pin_A, OUTPUT);
  pinMode(rote_pin_B, OUTPUT);
  ledcSetup(0, 20000, 8);
  ledcSetup(1, 20000, 8);
  ledcAttachPin(PWM_pin_A, 0);
  ledcAttachPin(PWM_pin_B, 1);

  //センサ初期化
  IMU.Init();

  //センサオフセット計算
  offset_cal();
}


void loop() {
  Blynk.run();

  //センサオフセット再計算
  if (digitalRead(button) == 0) {
    offset_cal();
  }


  //制御周期計算
  nowTime = micros();
  loopTime = nowTime - oldTime;
  oldTime = nowTime;
  dt = (float)loopTime / 1000000.0; //sec


  //カルマンフィルタでセンサ角度取得[deg]
  AngleI = get_theta_acc();
  OmegaI = get_theta_dot();
  kalAngleX = kalmanX.getAngle(get_theta_acc(), get_theta_dot(), dt);
  AngleI = kalAngleX;
  //Serial.print(AngleI); Serial.print(",");
  //Serial.print(kalAngleX); Serial.print(",");
  //Serial.print(OmegaI); Serial.print(",");


  //角速度(OmegaI)の絶対値がある値(ここでは5)より小さいときはそこを基準(角速度0)とする。
  if ( abs( OmegaI ) < 3 ) {
    OmegaI = 0;
  }
  recOmegaI[0] = OmegaI;    //今の角速度をrecordしておく

  //角速度の絶対値がある値(8)より小さかったら静止としてcountS++
  countS = 0;
  for (int i = 0 ; i < 10 ; i++ ) {
    if ( abs( recOmegaI[i] ) < 8 ) {
      countS++;
    }
  }

  //9回分のデータ(１つに3600μs = 3.6 ms)なので,36ms間静止していたら, とにかく0
  if ( countS > 9 ) {
    //kalAngleX = 0;
    target_Angle = kalAngleX;
    target_Omega = OmegaI;
    vE5 = 0;
    xE5 = 0;
    sumPower = 0;
    sumSumP = 0;
  }

  //10回分のデータを取るために, 1つずつずらして格納。
  for (int i = 9 ; i > 0 ; i-- ) {
    recOmegaI[ i ] = recOmegaI[ i - 1 ];
  }

  /*-------------------------------------------------------------------------*/
  //PD制御
  power = ( kAngle * (AngleI-target_Angle) / 10 )  /* k1×躯体の傾き(角度)*/
          + ( kOmega * (OmegaI-target_Omega) / 10 )   /* k2×躯体の変化率(角速度)*/
          + ( kSpeed * vE5 / 10000 )    /* k3×車輪軸の移動速度*/
          + ( kDistance * xE5 / 10000 );/* k4×車輪軸の移動距離(ほぼ影響なし)*/

  //debug
  //Serial.print(kAngle * AngleI / 100); Serial.print(",");
  //Serial.print(kOmega * OmegaI / 100); Serial.print(",");
  //Serial.print(kSpeed * vE5 / 1000); Serial.print(",");
  //Serial.print(kDistance * xE5 / 1000); Serial.print(",");
  //Serial.println(power);


  //95*powerScale/100が, 255を超えたら255を, -255を下に超えたら-255を出力(ただの安全策)
  power = max ( min ( 90 * power / 100 , 255 ) , -255 );

  //後処理
  sumPower = sumPower + power;  //出力値の総和
  sumSumP = sumSumP + sumPower; //出力値の総和の総和(積分)
  vE5 = sumPower;               //出力値の総和 = 速度
  xE5 = sumSumP / 1000;         //出力値の総和の総和 = 移動距離
  /*-------------------------------------------------------------------------*/

  //倒立動作開始！！！
  if (power > 0) {
    ledcWrite(0, abs(power) + rightpower );
    digitalWrite(rote_pin_A, LOW);
    ledcWrite(1, abs(power) + leftpower );
    digitalWrite(rote_pin_B, HIGH);
  } else {
    ledcWrite(0, abs(power) + rightpower );
    digitalWrite(rote_pin_A, HIGH);
    ledcWrite(1, abs(power) + leftpower );
    digitalWrite(rote_pin_B, LOW);
  }
  delayMicroseconds(1800);
}


// =================
// blynk callback --
// =================
// V2-V5 param調整
BLYNK_WRITE(V2) {
  kAngle = param.asInt();
}

BLYNK_WRITE(V3) {
  kOmega = param.asInt();
}

BLYNK_WRITE(V4) {
  kSpeed = param.asInt();
}

BLYNK_WRITE(V5) {
  kDistance = param.asInt();
}

// 操縦用ジョイスティック
BLYNK_WRITE(V1)
{
  int x = param[0].asInt();
  int y = param[1].asInt();
  if (x > 137) {
    //左旋回
    rightpower = 1;
    leftpower = -1;
  } else if (x < 117) {
    //右旋回
    rightpower = -1;
    leftpower = 1;
  } else {
    //x軸が中央にある時、前進後退
    //前進
    if (y > 137) {
      rightpower = 1;
      leftpower = 1;
    } else if (y < 117) {
      //後進
      rightpower = -1;
      leftpower = -1;
    } else {
      //ストップ
      rightpower = 0;
      leftpower = 0;
    }
  }
}
