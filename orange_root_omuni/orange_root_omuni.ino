#include <PS4BT.h>
#include <usbhub.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
#include <math.h>

/*
  pwm_pin dir_pinの中身
  [0] 右前のモータ
  [1] 左前のモータ
  [2] 左後ろのモータ
  [3] 右後ろのモータ
*/

int pwm_pin[] = {6, 5, 4, 8};
int dir_pin[] = {44, 38, 32, 26};
//int motor_pin[] = [pwm_pin, dir_pin];
//int motor_pin[][] = {
//  {6,2,4,5},
//  {44,26,32,28}
//};

//typedef struct {
//  int pwm_pin;
//  int dir_pin;
//} motor_pin;
//
//const motor_pin Omuni[] {
//  {6, 44},
//  {2, 26},
//  {4, 32},
//  {5, 38}
//};



double Lx, Ly, Rx, Ry;
uint8_t Lpower, Rpower;
double motors[4];
double motor_power;
double rad;

USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd, PAIR);//初期接続
//PS4BT PS4(&Btd);//二回目以降

double angleTorad(double angle)
{
  return angle * PI / 180.0;
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double get_power(int x, int y)
{
  return sqrt(pow(x, 2) + pow(y, 2));
}
void setup() {
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial);
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1);
  }
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(dir_pin[i], OUTPUT);
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));
}


void loop() {
  // loopを回すよりwhileのほうがはやい
  while (true) {
    Usb.Task();
    if (PS4.connected()) {
      Lx = double(PS4.getAnalogHat(LeftHatX) - 127);
      Ly = double((PS4.getAnalogHat(LeftHatY) - 127) * -1);
      Rx = double(PS4.getAnalogHat(RightHatX) - 127);
      Ry = double((PS4.getAnalogHat(RightHatY) - 127) * -1);
      rad = atan2(Ly, Lx);
      if (Lx > 25.0 && Lx < -25.0 && Ly > 25.0 && Ly < -25.0) {
        continue;
      }
      Lx = mapf(double(Lx), 0.0, 100.0, 0.0, 90.0);
      Ly = mapf(double(Ly), 0.0, 100.0, 0.0, 90.0);
      Rx = mapf(double(Rx), 10.0, 127.0, 0.0, 1.0);
      // Ry = mapf(double(Ry), 10.0, 127.0, 0.0, 1.0);
      for (uint8_t i = 0;i < 4;i++){
        motors[i] = (cos(angleTorad(135.0 + (90.0 * double(i)))) * Lx) + (sin(angleTorad(135.0 + (90.0 * double(i)))) * Ly);

        if (motors[i] > 0.0) {
          digitalWrite(dir_pin[i], LOW);
        } else {
          digitalWrite(dir_pin[i], HIGH);
        }

        motor_power = abs(motors[i]);
        analogWrite(pwm_pin[i], int(motor_power));
      }
    }
  }
}
