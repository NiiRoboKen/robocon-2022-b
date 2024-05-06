#include <SPI.h>
#include <math.h>

void setup()
{
  Serial.begin(19200);
  Serial.print("conected");
}

void loop()
{
  while (true) {
    // とりあえずこんなかんじに書いとけば動く補正つけてる角度の詳しくはノートのはしをみればわかる
    double Lx = 0.0;
    double Ly = 127.0;
    double motors[4];
    double map_power;
    double rad = atan2(Ly, Lx);
    Serial.print("rad = ");
    Serial.println(rad);
    if (abs(Lx) < 10.0 && abs(Ly) < 10.0) {
      continue;
    }
    Lx = mapf(double(Lx), 0.0, 100.0, 0.0, 90.0);
    Serial.print("Lx: ");
    Serial.println(Lx);
    Ly = mapf(double(Ly), 0.0, 100.0, 0.0, 90.0);
    Serial.print("Ly: ");
    Serial.println(Ly);
    // Rx = mapf(double(Rx), 10.0, 127.0, 0.0, 1.0);
    // Ry = mapf(double(Ry), 10.0, 127.0, 0.0, 1.0);
    for (uint8_t i = 0; i < 4; i++) {
      Serial.println("=====================================");
      motors[i] = (cos(angleTorad(135.0 + (90.0 * double(i)))) * Lx) + (sin(angleTorad(135.0 + (90.0 * double(i)))) * Ly);
      Serial.println(cos(angleTorad(135 + (90.0 * double(i)))) * Lx);
      Serial.println(sin(angleTorad(135 + (90.0 * double(i)))) * Ly);
      // motors[i] = mapf(abs(motors[i]), 0.0, 141.0);
      Serial.println(motors[i]);// debugprint
      double tmp = (rad + (PI / 4.0)) / (PI / 2.0);
      tmp = tmp - int(tmp);
      Serial.println(tmp);
      double tmp2 = 0.5 - abs(tmp - 0.5);
      double tmp3 = mapf(tmp2, 0.0, 0.5, 0.0, 46.0); // 46は補正値
      tmp3 = mapf(abs(motors[i]), 0.0, 80.0, 0.0, 1.0) * tmp3;
      Serial.print("tmp = ");
      Serial.println(tmp3);
      motors[i] = tmp3 + abs(motors[i]); // こいつはdegitalWriteのあとにする
      // Serial.println(mapf(tmp, 0.0, 0.5, 0.0, 30.0) + 90);
      // map_power = mapf(abs(motors[i]), 0.0, 0.71, 0.0, 1.0);
      // motors[i] = motors[i] / 2;
      // Serial.println(motors[i]);
      // map_power = mapf((motors[i]), 0.0, 0.71, 0.0, 255.0);
      //if (map_power > 1.0) {
      //  map_power = 1.0;
      //}
      // map_power = mapf(abs(motors[i]), 0.0, 1.0, 0.0, 255.0);
      Serial.print(i);
      Serial.print(": ");
      Serial.println(motors[i]);
    }
  }
}

double angleTorad(double angle)
{
  return angle * PI / 180.0;

}

void angleTorad_test()
{
  for (int i = 0; i < 4; i++) {
    Serial.println(cos(angleTorad(135 + (90 * i))));
  }
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
