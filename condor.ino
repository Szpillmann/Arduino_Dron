#include <math.h>
#include <BMP280_DEV.h>
#include <MPU9250.h>
#include <SoftwareSerial.h>
#include <Servo.h>
//bluetooth HC05 szpilman 1234
SoftwareSerial MyBlue(10, 11);  // RX | TX

//servos
Servo L_F_prop;
Servo L_B_prop;
Servo R_F_prop;
Servo R_B_prop;
float pwm_L_F, pwm_L_B, pwm_R_F, pwm_R_B;

//To store the 1000us to 2000us value we create variables and store each channel
int input_YAW = 0;          //In my case channel 4 of the receiver and pin D12 of arduino
int input_PITCH = 0;        //In my case channel 2 of the receiver and pin D9 of arduino
int input_ROLL = 0;         //In my case channel 1 of the receiver and pin D8 of arduino
int input_THROTTLE = 1100;  //In my case channel 3 of the receiver and pin D10 of arduino

// uso del BMP
int led_sapo = 32, altitud_BMP;

//bluetoot
char flag;

float temperature, pressure, altitudbar;
BMP280_DEV bmp280;

//Gyro Variables
float elapsedTime = 0, time = 0, timePrev = 0;  //Variables for time control
float Total_angle_x, Total_angle_y,Total_angle_z;

MPU9250 mpu;

//Averager
const int numReadingsp = 7, numReadingsy = 7,numReadingsz = 7;
int readingsp[numReadingsp], readingsy[numReadingsy],readingsz[numReadingsz];
int readIndexp = 0, readIndexy = 0,readIndexz = 0;
int totalp = 0, totaly = 0,totalz = 0;
int PromX, PromY,PromZ;

///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp = 0.057;  //3.55
double roll_ki = 0.03;   //0.003
double roll_kd = 0.01;   //2.05
float roll_desired_angle = 0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp = 0.057;  //3.55
double pitch_ki = 0.003;  //0.003
double pitch_kd = 0.01;   //2.05
float pitch_desired_angle = 0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double yaw_kp = 0.057;  //3.55
double yaw_ki = 0.003;  //0.003
double yaw_kd = 0.01;   //2.05
float yaw_desired_angle = 0;
//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p;
float pitch_pid_i;
float pitch_pid_d;
//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, roll_error, roll_previous_error;
float roll_pid_p;
float roll_pid_i;
float roll_pid_d;
//////////////////////////////PID FOR ROLL///////////////////////////
float yaw_PID, yaw_error, yaw_previous_error;
float yaw_pid_p;
float yaw_pid_i;
float yaw_pid_d;

// seteo de una vuelta
void setup() {
  Serial.begin(115200);
  MyBlue.begin(57600);  //Baud Rate for AT-command Mode.
  Wire.begin();
  Serial.println("MPU");
  bmp280.begin(0X76);
  mpu.setup(0x68);  // change to your own address
  bmp280.setTimeStandby(TIME_STANDBY_1000MS);
  bmp280.startNormalConversion();
  Serial.println("BMP y MPU set");
  Wire.beginTransmission(0x68);  //begin, Send the slave adress (in this case 68)
  Wire.write(0x6B);              //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);  //end the transmission
  //Gyro config
  Wire.beginTransmission(0x68);  //begin, Send the slave adress (in this case 68)
  Wire.write(0x1B);              //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);              //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);    //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68);  //Start communication with the address found during search.
  Wire.write(0x1C);              //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);              //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  Serial.println("calibrando accel");
  Serial.println("Accel Gyro calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();
  // Serial.println("Mag calibration will start in 5sec.");
  // Serial.println("Please Wave device in a figure eight until done.");
  // delay(5000);
  // mpu.calibrateMag();
  print_calibration();
  mpu.verbose(false);
  Serial.println("Wire");

  /*Here we calculate the acc data error before we start the loop
 * I make the mean of 200 values, that should be enough*/

  bmp280.getMeasurements(temperature, pressure, altitudbar);

  calibrarPROP();
  
  Serial.println("corriendo");
}
//constante ciclo
void loop() {
  time = millis();
  elapsedTime = (time - timePrev);
  if (elapsedTime >= 1000) {
    timePrev = time;
    ledsapo();
  }
  if (mpu.update()) {
      static uint32_t prev_ms = millis();
      if (millis() > prev_ms + 25) {
          Total_angle_z = mpu.getYaw();
          Total_angle_x = mpu.getPitch();
          Total_angle_y = mpu.getRoll();
          prev_ms = millis();
      }
  }
  bmp280.getMeasurements(temperature, pressure, altitudbar);

  if (MyBlue.available())
  flag = char(MyBlue.read());

  if (flag == '1') {
    if (input_THROTTLE > 1700)
    {input_THROTTLE = 1700;}
    else
    {input_THROTTLE = input_THROTTLE + 5;}
    ledsapo();
  }
  if (flag == '2') {
    if (input_THROTTLE < 1200)
    { input_THROTTLE = 1200;}
    else
    {input_THROTTLE = input_THROTTLE - 5;}
    ledsapo();
  }
  flag=0;

  totalp = totalp - readingsp[readIndexp];
  readingsp[readIndexp] = Total_angle_x;
  totalp = totalp + readingsp[readIndexp];
  readIndexp = readIndexp + 1;
  if (readIndexp >= numReadingsp) {
    readIndexp = 0;
  }
  PromX = totalp / numReadingsp;

  totalz = totalz - readingsz[readIndexz];
  readingsz[readIndexz] = Total_angle_z;
  totalz = totalz + readingsz[readIndexz];
  readIndexz = readIndexz + 1;
  if (readIndexz >= numReadingsz) {
    readIndexz = 0;
  }
  PromZ = totalz / numReadingsz;

  totaly = totaly - readingsy[readIndexy];
  readingsy[readIndexy] = Total_angle_y;
  totaly = totaly + readingsy[readIndexy];
  readIndexy = readIndexy + 1;
  if (readIndexy >= numReadingsy) {
    readIndexy = 0;
  }
  PromY = totaly / numReadingsy;

  roll_desired_angle = 0;
  pitch_desired_angle = 0;
  yaw_desired_angle = 0;

  roll_error = PromY - roll_desired_angle;
  pitch_error = PromX - pitch_desired_angle;
  yaw_error = PromZ - yaw_desired_angle;

  roll_pid_p = roll_kp * roll_error;
  pitch_pid_p = pitch_kp * pitch_error;
  yaw_pid_p = yaw_kp * yaw_error;

  if (-3 < roll_error < 3) {
    roll_pid_i = roll_pid_i + (roll_ki * roll_error);
  }
  if (-3 < pitch_error < 3) {
    pitch_pid_i = pitch_pid_i + (pitch_ki * pitch_error);
  }
  if (-3 < yaw_error < 3) {
    yaw_pid_i = yaw_pid_i + (yaw_ki * yaw_error);
  }

  roll_pid_d = roll_kd * ((roll_error - roll_previous_error) / elapsedTime);
  pitch_pid_d = pitch_kd * ((pitch_error - pitch_previous_error) / elapsedTime);
  yaw_pid_d = yaw_kd * ((yaw_error - yaw_previous_error) / elapsedTime);

  roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
  pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
  yaw_PID = yaw_pid_p + yaw_pid_i + yaw_pid_d;

  // if (roll_PID < -400) {
  //   roll_PID = -400;
  // }
  // if (roll_PID > 400) {
  //   roll_PID = 400;
  // }
  // if (pitch_PID < -400) {
  //   pitch_PID = -400;
  // }
  // if (pitch_PID > 400) {
  //   pitch_PID = 400;
  // }

  // writemicroseconds para 
  pwm_L_B = input_THROTTLE - roll_PID - pitch_PID + yaw_PID;
  pwm_L_F = input_THROTTLE - roll_PID + pitch_PID - yaw_PID;
  pwm_R_F = input_THROTTLE + roll_PID + pitch_PID + yaw_PID;
  pwm_R_B = input_THROTTLE + roll_PID - pitch_PID - yaw_PID;

  //  if (pwm_R_F < 1100)
  //  {
  //    pwm_R_F = 1100;
  //  }
  //  if (pwm_R_F > 2000)
  //  {
  //    pwm_R_F = 2000;
  //  }

  //  if (pwm_L_F < 1100)
  //  {
  //    pwm_L_F = 1100;
  //  }
  //  if (pwm_L_F > 2000)
  //  {
  //    pwm_L_F = 2000;
  //  }

  //  if (pwm_R_B < 1100)
  //  {
  //    pwm_R_B = 1100;
  //  }
  //  if (pwm_R_B > 2000)
  //  {
  //    pwm_R_B = 2000;
  //  }

  //  if (pwm_L_B < 1100)
  //  {
  //    pwm_L_B = 1100;
  //  }
  //  if (pwm_L_B > 2000)
  //  {
  //    pwm_L_B = 2000;
  //  }

  roll_previous_error = roll_error;
  pitch_previous_error = pitch_error;
  yaw_previous_error = yaw_error;

  // if (input_THROTTLE <= 800) {
  //   L_F_prop.writeMicroseconds(1000);
  //   L_B_prop.writeMicroseconds(1000);
  //   R_F_prop.writeMicroseconds(1000);
  //   R_B_prop.writeMicroseconds(1000);
  //   pwm_L_F = 1000;
  //   pwm_L_B = 1000;
  //   pwm_R_F = 1000;
  //   pwm_R_B = 1000;
  // } else {
  //   L_F_prop.writeMicroseconds(pwm_L_F);
  //   L_B_prop.writeMicroseconds(pwm_L_B);
  //   R_F_prop.writeMicroseconds(pwm_R_F);
  //   R_B_prop.writeMicroseconds(pwm_R_B);
  // }

  impresiones();
}
void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}
void calibrarPROP()
{
  L_F_prop.detach();
  L_B_prop.detach();
  R_F_prop.detach();
  R_B_prop.detach();

  L_F_prop.attach(4,0,180);
  Serial.println("attach");
  delay(1100);
  L_F_prop.write(map(2000, 1000, 2000, 0, 180));
  L_F_prop.write(map(1000, 1000, 2000, 0, 180));
  Serial.println("7000");
  delay(7000);

  R_F_prop.attach(1,0,180);
  Serial.println("attach");
  delay(1100);
  R_F_prop.write(map(2000, 1000, 2000, 0, 180));
  R_F_prop.write(map(1000, 1000, 2000, 0, 180));
  Serial.println("7000");
  delay(7000);

  R_B_prop.attach(2,0,180);
  Serial.println("attach");
  delay(1100);
  R_B_prop.write(map(2000, 1000, 2000, 0, 180));
  R_B_prop.write(map(1000, 1000, 2000, 0, 180));
  Serial.println("7000");
  delay(7000);

  L_B_prop.attach(3,0,180);
  Serial.println("attach");
  delay(1100);
  L_B_prop.write(map(2000, 1000, 2000, 0, 180));
  L_B_prop.write(map(1000, 1000, 2000, 0, 180));
  Serial.println("7000");
  delay(7000);
}
void impresiones()
{
  Serial.print("LB: ");
  Serial.print(pwm_L_B);
  Serial.print("   |   ");
  Serial.print("LF: ");
  Serial.print(pwm_L_F);
  Serial.print("   |   ");
  Serial.print("RF: ");
  Serial.print(pwm_R_F);
  Serial.print("   |   ");
  Serial.print("RB: ");
  Serial.print(pwm_R_B);

  Serial.print("flag ");
  Serial.print(flag);
  // Serial.print("   |   ");
  // Serial.print("L_F_prop: ");
  // Serial.print(L_F_prop.read());
  // Serial.print("   |   ");
  // Serial.print("L_B_prop: ");
  // Serial.print(L_B_prop.read());
  // Serial.print(" | ");
  // Serial.print("R_F_prop: ");
  // Serial.print(R_F_prop.read());
  // Serial.print(" | ");
  // Serial.print("R_B_prop: ");
  // Serial.print(R_B_prop.read());

  Serial.print("   |   ");
  Serial.print("PromX: ");
  Serial.print(PromX);
  Serial.print("   |   ");
  Serial.print("PromY: ");
  Serial.print(PromY);
  Serial.print(" | ");
  Serial.print("PromZ: ");
  Serial.print(PromZ);
  // Serial.print(" | ");
  // Serial.print("time: ");
  // Serial.print(time);
  // Serial.print(" | ");
  // Serial.print("timePrev: ");
  // Serial.print(timePrev);
  // Serial.print(" | ");
  // Serial.print("elapsedTime: ");
  // Serial.print(elapsedTime);
  Serial.print(" | ");
  Serial.print("input_THROTTLE: ");
  Serial.print(input_THROTTLE);
  Serial.print(" | ");

  if (temperature != 0) {
    Serial.print("temperature: ");
    Serial.print(temperature);
    Serial.print("   |   ");
    Serial.print("pressure: ");
    Serial.print(pressure);
    Serial.print("   |   ");
    Serial.print("altitudbar: ");
    Serial.print(altitudbar);
  }
  Serial.println("");
}
void ledsapo()
{
  if (digitalRead(led_sapo) == 1)
      digitalWrite(led_sapo, LOW);
    if (digitalRead(led_sapo) == 0)
      digitalWrite(led_sapo, HIGH);
}
