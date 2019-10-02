//You can download the library from github
#include <QTRSensors.h>
//array used - QTR-8RC Infra Red Reflectance Sensor Array

#define Kp 2 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 40// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) , in my case the Kd value is 20 times bigger than the Kp value 
#define MaxSpeed 255// max speed of the robot
#define BaseSpeed 255 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  8     // number of sensors used

#define speedturn 180

/*#define rightMotor1 3
#define rightMotor2 5
#define rightMotorPWM 8
#define leftMotor1 10
#define leftMotor2 11
#define leftMotorPWM 12
#define motorPower 9
*/
//You can also verify this by checking your desired connections

//Red Dual Motor Driver Module Shield Board 1a Tb6612fng is used for controlling motors 

int STBY = 8; //standby

//Motor A
int PWMA = 5; //This is for speed control
int AIN1 = 3; //Direction
int AIN2 = 4; //Direction

//Motor B
int PWMB = 11; //This is for speed control
int BIN1 = 12; //Direction
int BIN2 = 13; //Direction
QTRSensorsRC qtrrc((unsigned char[]) {A5,A4,A3,A2,A1,A0} ,NUM_SENSORS, 2500, QTR_NO_EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  /*pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPU T);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(motorPower, OUTPUT);
  */
pinMode(STBY, OUTPUT);

pinMode(PWMA, OUTPUT);
pinMode(AIN1, OUTPUT);
pinMode(AIN2, OUTPUT);

pinMode(PWMB, OUTPUT);
pinMode(BIN1, OUTPUT);
pinMode(BIN2, OUTPUT);

  delay(3000);
  
  int i;
  for (int i = 0; i < 100; i++) // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
  {
   //this is for automatic calibration 
    if ( i  < 25 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
    {
      move(1, 70, 1);
      move(0, 70, 0); 
    }
    else
    {
      move(1, 70, 0);
      move(0, 70, 1);  
    }
    qtrrc.calibrate();   
    delay(20);
  }
  wait();
  delay(3000); // wait for 3s to position the bot before entering the main loop 
}  

int lastError = 0;
unsigned int sensors[8];
int position = qtrrc.readLine(sensors);

void loop()
{  
  position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  
  if(position>6700){
    move(1, speedturn, 1);
    move(0, speedturn, 0);
    return;    
  }
  if(position<300){ 
    move(1, speedturn, 0);
    move(0, speedturn, 1);
    return;
  }
  
  int error = position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = BaseSpeed + motorSpeed;
  int leftMotorSpeed = BaseSpeed - motorSpeed;
  
  if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0)rightMotorSpeed = 0;    
  if (leftMotorSpeed < 0)leftMotorSpeed = 0;
    
  move(1, rightMotorSpeed, 1);
  move(0, leftMotorSpeed, 1);
}
  
void wait(){
  digitalWrite(STBY, LOW);
}

void move(int motor, int speed, int direction){
  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1=HIGH;
  boolean inPin2=LOW;
  
  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }  
  if(direction == 0){
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if(motor == 0){
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }
  if(motor == 1){
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }  
}
