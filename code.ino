#include "PinChangeInt.h"
#include "MsTimer2.h"
#include "Adeept_KalmanFilter.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define Ouput_MIN_LIMIT -255  //Min limit for PID 
#define Ouput_MAX_LIMIT 255  //Max limit for PID 
#define SAMPLE_TIME_IN_MILLI 5  //This is PID sample time in milliseconds
#define SETPOINT_ANGLE_OFFSET 0 // Balancing point
#define MIN_ABSOLUTE_SPEED 0  //Min motor speed
#define PID_KP 7 //8
#define PID_KI 0 //20
#define PID_KD 0.1  //0.09

MPU6050 mpu; //Instantiate an MPU1050 object with the object name mpu
Adeept_KalmanFilter kalmanfilter;//Instantiate an KalmanFilter object with the object name kalmanfilter

int16_t ax, ay, az, gx, gy, gz;

/********************angle data*********************/
float Q;
float Angle_ax; //The angle of inclination calculated from the acceleration
float Angle_ay;
float K1 = 0.05; // The weight of the accelerometer
float angle0 = 0.00; //Mechanical balance angle

/***************Kalman_Filter*********************/
float Q_angle = 0.001, Q_gyro = 0.005; //Angle data confidence, angular velocity data confidence
float R_angle = 0.5 , C_0 = 1;
float timeChange = 5; //Filter method sampling time interval milliseconds
float dt = timeChange * 0.001; //Note: The value of dt is the filter sampling time

/***************Controller parameter*********************/
float kp = PID_KP;
float ki = PID_KI;
float kd = PID_KP;
float preAngle = 0;
int outputSpeed = 0;
float integral=0,error=0,derivative=0,proportional=0;

//Motor pin
int enableMotor1=10;   //Motor left
int motor1Pin1=6;
int motor1Pin2=7;

int motor2Pin1=8;     //Motor right
int motor2Pin2=9;
int enableMotor2=5;

/*Interrupt timing 5ms timer interrupt*/
void interTimer2()
{
  sei(); 
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //IIC gets MPU1050 six axis data ax ay az gx gy gz
  kalmanfilter.angleTest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro,R_angle,C_0,K1);  //Get angle and Kaman filter  
  outputSpeed = pidController(kalmanfilter.angle,preAngle,0);
  preAngle = kalmanfilter.angle;
  rotateMotor(outputSpeed,outputSpeed);
}

void setupMotors()
{
  //Motor Left
  pinMode(enableMotor1,OUTPUT);
  pinMode(motor1Pin1,OUTPUT);
  pinMode(motor1Pin2,OUTPUT);
  //Motor right
  pinMode(enableMotor2,OUTPUT);
  pinMode(motor2Pin1,OUTPUT);
  pinMode(motor2Pin2,OUTPUT);
  rotateMotor(0,0);
}

void rotateMotor(int speed1, int speed2)
{
  if (speed2<-79) speed2-=30;
  if (speed2>79) speed2+=30;
  if (speed1 < 0)
  {
    digitalWrite(motor1Pin1,HIGH);
    digitalWrite(motor1Pin2,LOW);    
  }
  else if (speed1 >= 0)
  {
    digitalWrite(motor1Pin1,LOW);
    digitalWrite(motor1Pin2,HIGH);      
  }
  if (speed2 < 0)
  {
    digitalWrite(motor2Pin1,LOW);
    digitalWrite(motor2Pin2,HIGH);    
  }
  else if (speed2 >= 0)
  {
    digitalWrite(motor2Pin1,HIGH);
    digitalWrite(motor2Pin2,LOW);      
  }
  speed1 = abs(speed1) + MIN_ABSOLUTE_SPEED;
  speed2 = abs(speed2) + MIN_ABSOLUTE_SPEED;
  speed1 = constrain(speed1, MIN_ABSOLUTE_SPEED, 255);
  speed2 = constrain(speed2, MIN_ABSOLUTE_SPEED, 255);
  analogWrite(enableMotor1,speed1);
  analogWrite(enableMotor2,speed2);    
}

void setup() { 
  setupMotors();
  Wire.begin();
  mpu.initialize();
  delay(2);
  Serial.begin(115250); 
  delay(50);
 //5ms timer interrupt setting. Use timer2. Note: Using timer2 will affect the PWM output of pin3 and pin11.
 //Because the PWM is used to control the duty cycle timer, so when using the timer should pay attention to 
 // See the corresponding timer pin port.
  MsTimer2::set(5, interTimer2);
  MsTimer2::start();
}

void loop() 
{
  
}

int pidController(float curAngle, float preAngle, float setpoint)
{
  Serial.print("curAngle ");
  Serial.print(curAngle);
  Serial.print("preAngle ");
  Serial.print(preAngle);
  error = setpoint - curAngle;
  proportional = error;
  derivative = (preAngle-curAngle) / 0.005;
  int output = int(kp*proportional + kd*derivative);
  if (output > 230) output=230;
  else if (output < -230) output=-230;
  if (curAngle>40) output=0;
  if (curAngle<-40) output=0;
  //else if (-60<output && output<-130) output=-130;
  //else if (60<output && output<130) output=130;
  //else if (-60<output && output<60) output=0;
  //else if (0<output) output=250;
  //else if (0>output) output=-250;
  Serial.print("proportional ");
  Serial.print(kp*proportional);
  Serial.print("derivative ");
  Serial.print(kd*derivative);
  Serial.print("output ");
  Serial.println(output);
  return output;
}
