
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Servo.h"

MPU6050 mpu; //mpu default adress is 0x68
Servo myservo; // create servo object to control a servo

//MPU6050 variables
int16_t ay, az, gx; //initialize acceleration on y and z and gyroscope on x (pitch) reading
float dt, pg, pa; //initialize dt-time quantized, pg-pitch calculation from gyroscope, pa-pitch calculation from accelerometer
float p = 0.0; //initialialize p for common-filtered pitch
unsigned long t0, t; //initialize intial and current time
const unsigned long interval = 4000; //for 4 ms loop

//Servo variables
float aoa0 = 140; //initialize initial angle of attack
float aoa = 0; //initialize aoa-angle of attack, ca-correction angle

//PID variables
float Kp = 5.0; 
float Ki = 10.0;
float Kd = 0.1;
float sum = 0.0; //integral calculation
float dif = 0.0; //differential calculation
float pprev = 0.0; //e(t-dt)
float e = 0.0; //error

void setup() {
  /* Start I2C interface */
  Wire.begin(); //initializes communication bus at 100kHz
  Wire.setClock(400000); //unlocking 400KHz bus
  Wire.setTimeout(100); //instructs I2C bus to wait 100 ms before it freezes

  myservo.attach(9); // attached servo on pin 9

  Serial.begin(115200); //set baud rate at 115200

  /* Initialize MPU6050 */
  mpu.initialize();

  /* set initial time*/
  t0 = micros();
  /* set offset taken from MPU6050 calibration MPU_raw example, set it up again if a new calibration was done*/
  mpu.setXAccelOffset(-5513); //ay
  mpu.setZAccelOffset(10109); //az
  mpu.setYGyroOffset(-57); //gx
}

void loop() {
  t = micros();
  if (t - t0 >= interval) {
  /* find the change of time dt from iteration before t0 to now t*/
    dt = (float)(t-t0)/1000000.0;
    t0 = t;
    /* Read raw data from the MPU6050 module*/
    ay= mpu.getAccelerationY();
    az = mpu.getAccelerationZ();
    gx = mpu.getRotationX();
    /* convert the raw data to estimated pitch angle in degrees*/
    float gxdeg = (float)gx/131.0; // convert the raw data to angle units
    pg = pg + gxdeg*dt; // calculate estimated pitch angle from the gyroscope
    pa = atan2((float)ay, (float)az)*180.0/M_PI; //calculate estimated ptich angle from accelerometer 

    p = 0.98*(p + gxdeg*dt) + 0.02*pa;

    sum = sum + p*dt; //sum calculaltion
    dif = (p - pprev)/dt; //dif calculation
    e = Kp*p + Ki*sum + Kd*dif;
    pprev = p; //recording the previous pitch for next loop

    aoa = aoa0 + e;
    myservo.write(aoa);

    /* Print the data */
    //Serial.print(dif); Serial.print("\t");
    //Serial.print(sum); Serial.print("\t");
    Serial.print(t); Serial.print("\n");
  }
}
