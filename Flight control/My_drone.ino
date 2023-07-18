/********************** ALHASAN ALKHATIB ****************************/
/********************** 28/03/2019       ****************************/


#include <SPI.h>
#include <math.h>
#include <Servo.h>
#include <Wire.h>

/*MPU*/
#define PI 3.141592654
#define rad_to_deg  180/3.141592654;
#define ChipSelPin1 53

/*Ultrasonic*/
#define trigPin  6  // APM OUTPUT PIN 05
#define echoPin  3  // APM OUTPUT PIN 06

/*pusula*/
#define addressHMC 0x1E //0011110b, I2C 7bit address of HMC5883
#define declinationAngle 0.1003

/*MPU*/
unsigned long t = 0;
int time = 0;
float Ax, Ay, Az;
int Gx, Gy, Gz;
float loop_time;
float t1 = 0;
float t2 = 0;
float Acceleration_angle[3]; float acc;
float Gyro_angle[3];
float Total_angle[3];
float gyro_error; float acc_error;
float Gyro_error_y; float Gyro_error_x; float Gyro_error_z;
volatile float acc_error_x, acc_error_y, acc_error_z;
float gyro_roll_input=0;float gyro_pitch_input=0;float gyro_yaw_input=0;

/*Ultrasonic*/
long duration;
int distanceCm;
int Ultrasonic_buffer[50]; uint8_t counter_Ultrasonic_buffer = 0;

/*Pusula*/
uint8_t xl, xh, yl, yh, zl, zh;
float x, y, z;//triple axis data
float H1;

/*desired values*/
int desired_altitude = 0;
int desired_pitch = 0;
int desired_roll = 0;
int desired_yaw = 0;
int Throttle = 0;


/*Serial port*/
char data[3] = "000";

/*PID*/
float Kp_pitch = 1.3; float Ki_pitch = 0.04; float Kd_pitch = 4.0;
float Kp_roll = 1.3; float Ki_roll = 0.04; float Kd_roll = 4.0;
float Kp_yaw = 4; float Ki_yaw = 0.02; float Kd_yaw = 4.0;
float Kp_altitude = 0.1; float Ki_altitude = 0.002; float Kd_altitude = 4.0;

float error_pitch; float error_roll; float error_yaw; float error_altitude;
float PID_pitch; float PID_roll; float PID_yaw; float PID_altitude;

float P_altitude; float I_altitude; float D_altitude;
float P_pitch; float I_pitch; float D_pitch;
float P_roll; float I_roll; float D_roll;
float P_yaw; float I_yaw; float D_yaw;

float error_altitude_prev = 0;
float error_pitch_prev = 0;
float error_roll_prev = 0;
float error_yaw_prev = 0;


/*ESC MOTOR*/
Servo L_F_MOTOR;
Servo L_B_MOTOR;
Servo R_F_MOTOR;
Servo R_B_MOTOR;

int ESC_PWM_1, ESC_PWM_2, ESC_PWM_3, ESC_PWM_4;

/*Emergency button*/
#define interrupt_Pin  2        // INT.0 PE4   - output 07 on APM - 
volatile bool Emergaecy_stop_flag = 0;
float loop_full_test = 0;
float time_test = 0;
unsigned long  Emergancy_time=0;

/*----battary----*/
float Bat_drop;
int Bat_analog_value;


/////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  delay(15000);
  Serial.begin(115200);

  /*Emergency button*/
  pinMode(interrupt_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interrupt_Pin), Emergency_button, FALLING); // trigger for when the pin goes from high to low.

  /*Ultrasonic*/
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  /*MPU*/
  pinMode(40, OUTPUT);
  digitalWrite(40, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  delay(100);
  pinMode(ChipSelPin1, OUTPUT);
  ConfigureMPU6000();  // configure chip
  delay(100);
  calibrate_baslangic_hatasi();
  Serial.print(acc_error_x);     Serial.print(" BB ");
  Serial.print(acc_error_y);     Serial.print(" BB ");
  delay(5000);

  /*Pusula*/
  Wire.begin();
  Wire.beginTransmission(addressHMC); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

  /*ESC MOTORS*/
  pinMode(12, OUTPUT);  //OUTPUT 01
  pinMode(11, OUTPUT);  //OUTPUT 02
  pinMode(8, OUTPUT);   //OUTPUT 03
  pinMode(7, OUTPUT);   //OUTPUT 04

  R_B_MOTOR.attach(12);  //RIGHT BACK motor  01
  R_F_MOTOR.attach(11); //RIGH FRONT motor   02
  L_F_MOTOR.attach(8);   //LEFT FRONT motor  03
  L_B_MOTOR.attach(7);   //LEFT BACK motor   04

  L_F_MOTOR.writeMicroseconds(1000);
  L_B_MOTOR.writeMicroseconds(1000);
  R_F_MOTOR.writeMicroseconds(1000);
  R_B_MOTOR.writeMicroseconds(1000);

  delay(1000);

  t1 = micros();
  t2 = micros();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
   loop_time = (t2 - t1) / 1000000.0;  // in second
   t1 = micros();  // actual time read
   
   time_test = loop_time;
   loop_full_test += time_test;                 // for Emergancy mode
   
   if (Serial.available()) // Chek for availablity of data at Serial Port
   {
      data[0] = Serial.read();
      data[1] = Serial.read();
      data[2] = Serial.read();
   }
   set_desired_values();                                               //Set desired values that determined from RP3 serial port
   MPU_read();                                                         // Read Gyro and Acc values
   
   //Ultrasonic_read();
   //HMC5883_read();
   Serial.print(loop_time * 1000);     Serial.print("  ");
   Serial.print( Acceleration_angle[0]);     Serial.print("  ");
   Serial.print( Acceleration_angle[1]);     Serial.print("  ");
   /*Serial.print( Total_angle[0]);     Serial.print("  ");
   Serial.print( Total_angle[1]);     Serial.print("  ");
   Serial.print(distanceCm);         Serial.print("  ");*/
   
   if (mean(Ultrasonic_buffer, 10) >= 2000) {
      Serial.println(" Warrnnig !! ");
      desired_altitude = 1;
   }
   if (loop_full_test < 15) {
      desired_altitude = 90;
   }
   else {
      desired_altitude = 1;
      //Serial.print(" STOOOOOOOOOOOOOOP !! ");   
   }
   
   PID_GAIN_Calculate();
   
   if (Throttle > 1600)Throttle = 1600;
   if (Throttle < 1100)Throttle = 1100;
   
   Throttle = 1200;
   /*Serial.print(Throttle);  Serial.print("  ");*/
   //PID_yaw=0;
   //PID_pitch=0;
   //PID_roll=0;
   ESC_PWM_1 = Throttle + PID_pitch - PID_roll - PID_yaw;
   ESC_PWM_2 = Throttle + PID_pitch + PID_roll + PID_yaw;
   ESC_PWM_3 = Throttle - PID_pitch + PID_roll - PID_yaw;
   ESC_PWM_4 = Throttle - PID_pitch - PID_roll + PID_yaw;
   
   /*Bat drop*/
   /* Bat_analog_value = analogRead(A5);
   Bat_drop = 5 * Bat_analog_value / 1024.0;
   Bat_drop = 3.225*Bat_drop;     //Gerlilim bolucu ile 3,225 oranı  
   if (Bat_drop<10.5)Bat_drop=10.5;
   if (Bat_drop>12.6)Bat_drop=12.6;
   //Serial.print(Bat_drop);         Serial.print("  ");*/
   Bat_drop=11.7;
   Bat_drop = 12.6 / Bat_drop;
   ESC_PWM_1 = ESC_PWM_1*Bat_drop;
   ESC_PWM_2 = ESC_PWM_2*Bat_drop;
   ESC_PWM_3 = ESC_PWM_3*Bat_drop;
   ESC_PWM_4 = ESC_PWM_4*Bat_drop;
   
   if (ESC_PWM_1 > 1800)ESC_PWM_1 = 1800;
   if (ESC_PWM_1 < 1100)ESC_PWM_1 = 1100;
   
   if (ESC_PWM_2 > 1800)ESC_PWM_2 = 1800;
   if (ESC_PWM_2 < 1100)ESC_PWM_2 = 1100;
   
   if (ESC_PWM_3 > 1800)ESC_PWM_3 = 1800;
   if (ESC_PWM_3 < 1100)ESC_PWM_3 = 1100;
   
   if (ESC_PWM_4 > 1800)ESC_PWM_4 = 1800;
   if (ESC_PWM_4 < 1100)ESC_PWM_4 = 1100;
   
   if(Emergaecy_stop_flag || loop_full_test > 30) {
      R_B_MOTOR.writeMicroseconds(0);
      R_F_MOTOR.writeMicroseconds(0);
      L_F_MOTOR.writeMicroseconds(0);
      L_B_MOTOR.writeMicroseconds(0);  
      //Serial.print("STOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP ABIIIIIIIIIIII  ");  
   }
   else{
      Serial.print(ESC_PWM_1);         Serial.print("  ");
      Serial.print(ESC_PWM_2);         Serial.print("  ");
      Serial.print(ESC_PWM_3);         Serial.print("  ");
      Serial.print(ESC_PWM_4);         Serial.print("  ");
      R_B_MOTOR.writeMicroseconds(ESC_PWM_1);
      R_F_MOTOR.writeMicroseconds(ESC_PWM_2);
      L_F_MOTOR.writeMicroseconds(ESC_PWM_3);
      L_B_MOTOR.writeMicroseconds(ESC_PWM_4);
   }
   
   //Serial.println("  ");
   while(micros() - t1 < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
   t2 = micros();  // actual time read
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Emergency_button() {

 // if (Emergaecy_stop_flag ) {
 //   Emergaecy_stop_flag = false;
 // }
 // else {
    Emergaecy_stop_flag = true;
 // }
 // delay(100);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PID_GAIN_Calculate() {
  error_altitude = desired_altitude - distanceCm;
  P_altitude = Kp_altitude * error_altitude;
  I_altitude = I_altitude + Ki_altitude * error_altitude;
  D_altitude = Kd_altitude * ((error_altitude - error_altitude_prev));
  
  if(I_altitude>400)I_altitude=400;
  if(I_altitude<-400)I_altitude=-400;
  Throttle = 1100 + P_altitude + I_altitude + D_altitude;
  error_altitude_prev = error_altitude;
  
  error_pitch = desired_pitch - gyro_pitch_input;
  P_pitch = Kp_pitch * error_pitch;
  I_pitch = I_pitch + Ki_pitch * error_pitch;
  D_pitch = Kd_pitch * ((error_pitch - error_pitch_prev));

  if(I_pitch>400)I_pitch=400;
  if(I_pitch<-400)I_pitch=-400;
  PID_pitch = P_pitch + I_pitch + D_pitch;
  if(PID_pitch>400)PID_pitch=400;
  if(PID_pitch<-400)PID_pitch=-400;
  error_pitch_prev = error_pitch;
    
  error_roll = desired_roll - gyro_roll_input;
  P_roll = Kp_roll * error_roll;
  I_roll = I_roll + Ki_roll * error_roll;
  D_roll = Kd_roll * ((error_roll - error_roll_prev));
 
  if(I_roll>400)I_roll=400;
   
  if(I_roll<-400)I_roll=-400;
   
  PID_roll = P_roll + I_roll + D_roll;
   
  if(PID_roll>400)PID_roll=400;
   
  if(PID_roll<-400)PID_roll=-400;
   
  error_roll_prev = error_roll;

  error_yaw = desired_yaw - gyro_yaw_input; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! what about the minus ???
  P_yaw = Kp_yaw * error_yaw;
  I_yaw = I_yaw + Ki_yaw * error_yaw;
  D_yaw = Kd_yaw * ((error_yaw - error_yaw_prev));
 
  if(I_yaw>400)I_yaw=400;
  if(I_yaw<-400)I_yaw=-400;
   
  PID_yaw = P_yaw + I_yaw + D_yaw;
   
  if(PID_yaw>400)PID_yaw=400;
  if(PID_yaw<-400)PID_yaw=-400;
  
  error_yaw_prev = error_yaw;
}

void set_desired_values() {
   
  if (data[0] == 'U' || data[1] == 'U' || data[2] == 'U') { //UP 
    if (Throttle>= 1600)Throttle=1600;
    else Throttle += 10;
     
    desired_pitch = 0;
    desired_roll = 0;
    desired_yaw = 0;
  }
  else if (data[0] == 'D' || data[1] == 'D' || data[2] == 'D') { //Down
    if (Throttle<= 100)Throttle=100;
    else Throttle -= 10;
     
    desired_pitch = 0;
    desired_roll = 0;
    desired_yaw = 0;
  }
    else if (data[0] == 'F' || data[1] == 'F' || data[2] == 'F') { //forward
       
    desired_pitch = 0;
    desired_roll = 10;
    desired_yaw = 0;
  }
  
  else if (data[0] == 'R' || data[1] == 'R' || data[2] == 'R') { //Right
    desired_pitch = 10;
    desired_roll = 0;
    desired_yaw = 0;
  }

  else if (data[0] == 'L' || data[1] == 'L' || data[2] == 'L') { //Left
    desired_pitch = -10;
    desired_roll = 0;
    desired_yaw = 0;
  }
  else if (data[0] == 'S' || data[1] == 'S' || data[2] == 'S') { //STOP to down
    Throttle = 10;
    desired_pitch = 0;
    desired_roll = 0;
    desired_yaw = 0;
  }
  else {
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HMC5883_read() {
  Wire.beginTransmission(addressHMC);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();

  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(addressHMC, 6);
  if (6 <= Wire.available()) {
    xl = Wire.read(); //X msb
    xh = Wire.read(); //X lsb
    yl = Wire.read(); //Y msb
    yh = Wire.read(); //Y lsb
    zl = Wire.read(); //Z msb
    zh = Wire.read(); //Z lsb 
  }
  x = (int16_t)(xl | ((int16_t)xh << 8));
  y = (int16_t)(yl | ((int16_t)yh << 8));
  z = (int16_t)(zl | ((int16_t)zh << 8));

  // Sakarya'nin manyetik sapması 5 derece, 42' --> yaklasık olarak 0,1003 radyan

  H1 = atan2(y, x);
  H1 += declinationAngle;
  if (H1 < 0)
    H1 += 2 * PI;
  H1 = H1 * 180 / PI;

  //the return value is H1
}

void Ultrasonic_read() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
   
  duration = pulseIn(echoPin, HIGH);
  distanceCm = duration * 0.034 / 2;
  Ultrasonic_buffer[counter_Ultrasonic_buffer++] = distanceCm;
   
  if (counter_Ultrasonic_buffer == 50)counter_Ultrasonic_buffer = 0;
}

int mean(int *matrix, int mlength) {
  int sum = 0;
  for (int i = 0; i < mlength; i++) {
    sum += matrix[i];
  }
  return sum / mlength;
}

void MPU_read()
{
  // 8g ---> must divide by 4096
  Ax = AcceX(ChipSelPin1)/4096.0 ;
  Ay = AcceY(ChipSelPin1)/4096.0 ;
  Az = AcceZ(ChipSelPin1)/4096.0 ;
  Gx = GyroX(ChipSelPin1) - Gyro_error_x;  // 
  Gy = GyroY(ChipSelPin1) - Gyro_error_y;  // 
  Gz = GyroZ(ChipSelPin1) - Gyro_error_z;  // 

  gyro_roll_input = (gyro_roll_input * 0.7) + ((Gy / 65.5) * 0.3);    //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((Gx / 65.5) * 0.3);  //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((Gz / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  Acceleration_angle[0]  = atan( Ay / sqrt(pow(( Ax ), 2) + pow(( Az), 2)))*rad_to_deg ;
  Acceleration_angle[1]  = atan(-1* Ax / sqrt(pow(( Ay ), 2) + pow(( Az), 2)))*rad_to_deg  ;
  Acceleration_angle[0] -= acc_error_x;
  Acceleration_angle[1] -= acc_error_y;

  /*---X---*/
  Gyro_angle[0] += loop_time*(Gx /( 65.5) );
  /*---Y---*/
  Gyro_angle[1] += loop_time*(Gy / (65.5) );
  /*---Z---*/
  Gyro_angle[2] =(Gz / (65.5) );

  Gyro_angle[0] +=Gyro_angle[1] * sin (loop_time * (Gz / (65.5)) * (3.14/180.0) ) ;
  /*---Y---*/
  Gyro_angle[1] -=Gyro_angle[0] * sin (loop_time * (Gz / (65.5)) * (3.14/180.0) );
  /*---Z---*/  

  Gyro_angle[0] = 0.996 * Gyro_angle[0] + 0.004*Acceleration_angle[0];
  Gyro_angle[1] = 0.996 * Gyro_angle[1] + 0.004*Acceleration_angle[1];
  Gyro_angle[2] = Gyro_angle[2];

  /*---X axis angle---*/
  Total_angle[0] = 0.9*(Total_angle[0]) + 0.1 * Gyro_angle[0];
  /*---Y axis angle---*/
  Total_angle[1] = 0.9*(Total_angle[1]) + 0.1 * Gyro_angle[1];
  /*---YAW---*/
  Total_angle[2] = 0.9*(Total_angle[2]) + 0.1 * Gyro_angle[2];
}

void calibrate_baslangic_hatasi() {

   //Now in order to obtain the gyro data in degrees/seconda we have to divide first
   //the raw value by 131 because that's the value that the datasheet gives us
   for (int i = 0; i < 2000; i++) {
      Gx = GyroX(ChipSelPin1);
      Gy = GyroY(ChipSelPin1);
      Gz = GyroZ(ChipSelPin1);
      Gyro_angle[0] += Gx;
      Gyro_angle[1] += Gy;
      Gyro_angle[2] += Gz;
   }
   
   for (int i = 0; i < 200; i++) {
      Ax = AcceX(ChipSelPin1)/4096.0;
      Ay = AcceY(ChipSelPin1)/4096.0;
      Az = AcceZ(ChipSelPin1)/4096.0;
      Acceleration_angle[0] += Ax ;
      Acceleration_angle[1] += Ay ;
      Acceleration_angle[2] += Az ;
   }

   Acceleration_angle[0]=Acceleration_angle[0]/200;
   Acceleration_angle[1]=Acceleration_angle[1]/200;
   Acceleration_angle[2]=Acceleration_angle[2]/200;
   acc_error_x  = atan( Acceleration_angle[1] / sqrt(pow(( Acceleration_angle[0]), 2) + pow(( Acceleration_angle[2]), 2)))*rad_to_deg ;
   acc_error_y = atan(-1 * ( Acceleration_angle[0]) / sqrt(pow(( Acceleration_angle[1]), 2) + pow(( Acceleration_angle[2]), 2)))*rad_to_deg;
   
   Gyro_error_x = Gyro_angle[0] / 2000;
   Gyro_error_y = Gyro_angle[1] / 2000;
   Gyro_error_z = Gyro_angle[2] / 2000;
   
   Gyro_angle[0] = 0;
   Gyro_angle[1] = 0;
   Gyro_angle[2] = 0;
   Acceleration_angle[0] = 0 ;
   
   Acceleration_angle[1] = 0 ;
   Acceleration_angle[2] = 0 ;
}

void SPIwrite(byte reg, byte data, int ChipSelPin) {
  uint8_t dump;
  digitalWrite(ChipSelPin, LOW);
  dump = SPI.transfer(reg);
  dump = SPI.transfer(data);
  digitalWrite(ChipSelPin, HIGH);
}

uint8_t SPIread(byte reg, int ChipSelPin) {
  uint8_t dump;
  uint8_t return_value;
  uint8_t addr = reg | 0x80;
  digitalWrite(ChipSelPin, LOW);
  dump = SPI.transfer(addr);
  return_value = SPI.transfer(0x00);
  digitalWrite(ChipSelPin, HIGH);
  return(return_value);
}

int AcceX(int ChipSelPin) {
  uint8_t AcceX_H = SPIread(0x3B, ChipSelPin);
  uint8_t AcceX_L = SPIread(0x3C, ChipSelPin);
  int16_t AcceX = AcceX_H << 8 | AcceX_L;
  return(AcceX);
}

int AcceY(int ChipSelPin) {
  uint8_t AcceY_H = SPIread(0x3D, ChipSelPin);
  uint8_t AcceY_L = SPIread(0x3E, ChipSelPin);
  int16_t AcceY = AcceY_H << 8 | AcceY_L;
  return(AcceY);
}

int AcceZ(int ChipSelPin) {
  uint8_t AcceZ_H = SPIread(0x3F, ChipSelPin);
  uint8_t AcceZ_L = SPIread(0x40, ChipSelPin);
  int16_t AcceZ = AcceZ_H << 8 | AcceZ_L;
  return(AcceZ);
}

int GyroX(int ChipSelPin) {
  uint8_t GyroX_H = SPIread(0x43, ChipSelPin);
  uint8_t GyroX_L = SPIread(0x44, ChipSelPin);
  int16_t GyroX = GyroX_H << 8 | GyroX_L;
  return(GyroX);
}

int GyroY(int ChipSelPin) {
  uint8_t GyroY_H = SPIread(0x45, ChipSelPin);
  uint8_t GyroY_L = SPIread(0x46, ChipSelPin);
  int16_t GyroY = GyroY_H << 8 | GyroY_L;
  return(GyroY);
}

int GyroZ(int ChipSelPin) {
  uint8_t GyroZ_H = SPIread(0x47, ChipSelPin);
  uint8_t GyroZ_L = SPIread(0x48, ChipSelPin);
  int16_t GyroZ = GyroZ_H << 8 | GyroZ_L;
  return(GyroZ);
}

void ConfigureMPU6000()
{
  // DEVICE_RESET @ PWR_MGMT_1, reset device
  SPIwrite(0x6B, 0x80, ChipSelPin1);
  delay(150);

  // TEMP_DIS @ PWR_MGMT_1, wake device and select GyroZ clock
  SPIwrite(0x6B, 0x03, ChipSelPin1);
  delay(150);

  // I2C_IF_DIS @ USER_CTRL, disable I2C interface
  SPIwrite(0x6A, 0x10, ChipSelPin1);
  delay(150);

  // SMPRT_DIV @ SMPRT_DIV, sample rate at 1000Hz
  SPIwrite(0x19, 0x00, ChipSelPin1);
  delay(150);

  // DLPF_CFG @ CONFIG, digital low pass filter at 42Hz
  SPIwrite(0x1A, 0x03, ChipSelPin1);
  delay(150);

  // FS_SEL @ GYRO_CONFIG, gyro scale at 500dps
  SPIwrite(0x1B, 0x08, ChipSelPin1);
  delay(150);

  // AFS_SEL @ ACCEL_CONFIG, accel scale at 8g (1g=8192)
  SPIwrite(0x1C, 0x10, ChipSelPin1);
  delay(150);
}
