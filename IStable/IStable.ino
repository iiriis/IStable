
#include "Wire.h"
#include <MPU6050_light.h>

#define alpha 0.5



float xang = 0.0;
MPU6050 mpu(Wire);

long ps = 0;

int bs=17, th=220, errorth=1;
static volatile float error = 0.0, pid = 0.0, lerror = 0.0, terror = 0.0, sp = 91.50, kp = 4, kd = 0.55, ki = 0.97;


void IMUsetup()
{
  Wire.begin();
  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  while (status != 0) { } 

  Serial.println("Calculating offsets, do not move MPU6050");
  delay(1000);
  mpu.calcOffsets();
  Serial.println("Done!\n");
}

void setup() {
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(7, OUTPUT);

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  Serial.begin(115200);
  IMUsetup();


}






void loop()
{
  
  long ps = millis();

  mpu.update();

  float xraw = mpu.getAngleX();
  double dang = mpu.getGyroX();

  xang = alpha * xraw + (1 - alpha) * xang;
  
  error = sp - xang;

  pid =  kp * error - kd * dang + ki * terror;

  terror += error;

  if (terror > 255)
    terror = 255;

  else if (terror < -255)
    terror = -255;

  if (xang < 60.0 || xang > 120.0)
  {
    rightMot(0);
    leftMot(0);
  }

  else
  {
    if (abs(error)<errorth)
      leftMot(pid);

    else
    {
      rightMot(pid);
      leftMot(pid);
    }
  }




  while (Serial.available() > 0)
  {
    char x = Serial.read();
    String dat = "";

    if (x == 's')
    {
      while (Serial.available() > 0)
        dat += (char)Serial.read();

      sp = dat.toDouble();
      printstats();
      break;
    }

    else if (x == 'p')
    {

      while (Serial.available() > 0)
        dat += (char)Serial.read();

      kp = dat.toDouble();
      printstats();
      break;
    }

    else if (x == 'i')
    {
      while (Serial.available() > 0)
        dat += (char)Serial.read();

      ki = dat.toDouble();

      printstats();
      break;
    }

    else if (x == 'd')
    {
      while (Serial.available() > 0)
        dat += (char)Serial.read();

      kd = dat.toDouble();

      printstats();
      break;
    }

    else if (x == 'b')
    {
      while (Serial.available() > 0)
        dat += (char)Serial.read();

      bs = dat.toInt();
      th=250-bs;
      printstats();
      break;
    }


    else if (x == 'e')
    {
      while (Serial.available() > 0)
        dat += (char)Serial.read();

      errorth = dat.toInt();
      printstats();
      break;
    }


  }

}


void printstats()
{
  Serial.print(sp);
  Serial.print(" ");
  Serial.print(kp);
  Serial.print(" ");
  Serial.print(ki);
  Serial.print(" ");
  Serial.print(kd);
  Serial.print(" ");
  Serial.print(errorth);
  Serial.print(" ");
  Serial.println(bs);
}

void rightMot(int speed)
{

  if (speed < -th)
    speed = -th;
  if (speed > th)
    speed = th;

  if (speed > 0)
  {
    digitalWrite(A1, HIGH);
    digitalWrite(A0, LOW);
    analogWrite(9, bs + speed);
  }
  else if (speed < 0)
  {
    digitalWrite(A1, LOW);
    digitalWrite(A0, HIGH);
    analogWrite(9, bs + speed * -1);
  }
  else
  {
    digitalWrite(9, LOW);
    digitalWrite(A1, LOW);
    digitalWrite(A0, LOW);
  }

}

void leftMot(int speed)
{
  if (speed < -th)
    speed = -th;
  if (speed > th)
    speed = th;

  if (speed > 0)
  {
    digitalWrite(12, HIGH);
    digitalWrite(7, LOW);
    analogWrite(10, bs + speed);
  }
  else if (speed < 0)
  {
    digitalWrite(12, LOW);
    digitalWrite(7, HIGH);
    analogWrite(10, bs + speed * -1);
  }
  else
  {
    digitalWrite(10, LOW);
    digitalWrite(12, LOW);
    digitalWrite(7, LOW);
  }

}


