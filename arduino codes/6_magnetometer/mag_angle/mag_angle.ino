#include <Wire.h>
#define MPU_addr 0x68 // 0x69
#define MAG_5883_addr 0x0D
float AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float AcX_P,AcY_P,AcZ_P;
float SampleFrequency = 250, dt = 1/SampleFrequency, loop_timer = 1000000*dt;
float acc_sensitivity=4096;
float amx, amy, amz;
float phi, th;
int MgX, MgY, MgZ;
float Vx = 226.04, Vy = 308.03, Vz = 40.425;
float phi_mag;
unsigned long t;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  TWBR = 12;     
  setup_MPU_6050_registers();
  setup_MAG_DA5883_registers();
  t=micros();
}

void loop() {  
  read_MPU_6050_data();
  read_MAG_DA5883_data();
  //LPF_acc();
  acc_angle_calc();
  mag_angle_calc();  
  print_angles();
  
  //Serial.println((micros()-t));
  delay(50);
}

void LPF_acc(){
  float k = 0.05;
  AcX = AcX*k + (1-k)*AcX_P;
  AcY = AcY*k + (1-k)*AcY_P;
  AcZ = AcZ*k + (1-k)*AcZ_P;
  AcX_P = AcX;
  AcY_P = AcY;
  AcZ_P = AcZ;
}

void mag_angle_calc(){
  MgX = MgX - Vx;
  MgY = MgY - Vy;
  MgZ = MgZ - Vz;

  float BnX = MgX*cos(th) + MgY*sin(th)*sin(phi) + MgZ*sin(th)*cos(phi);
  float BnY = MgY*cos(phi) - MgZ*sin(phi);
  phi_mag = -atan2(BnY,BnX);
}

void acc_angle_calc(){
  amx = AcX/acc_sensitivity;
  amy = AcY/acc_sensitivity;
  amz = AcZ/acc_sensitivity;
  
  float g = sqrt(amx*amx + amy*amy + amz*amz);
  phi = atan2(amy,amz);
  th = -asin(amx/g);
}

void print_angles(){
  Serial.print(phi*180/PI,3); Serial.print("\t");
  Serial.print(th*180/PI,3) ; Serial.print("\t");
  Serial.println(phi_mag*180/PI,3);
}

void setup_MPU_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(MPU_addr);                                    //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission(true);                                          //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(MPU_addr);                                    //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission(true);                                          //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(MPU_addr);                                    //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission(true);                                          //End the transmission
  //low pass filter
  Wire.beginTransmission(MPU_addr);                                    //Start communicating with the MPU-6050
  Wire.write(0x1A);                                                    //Send the requested starting register
  Wire.write(0x03);                                                    //Set the requested starting register
  Wire.endTransmission(true); 
}

void setup_MAG_DA5883_registers(){
  Wire.beginTransmission(MAG_5883_addr);
  Wire.write(0x0B); // Tell the HMC5883 to Continuously Measure
  Wire.write(0x01); // Set the Register
  Wire.endTransmission();
  Wire.beginTransmission(MAG_5883_addr);
  Wire.write(0x09); // Tell the HMC5883 to Continuously Measure
  Wire.write(0x1D); // Set the Register
  Wire.endTransmission();
}

void read_MPU_6050_data(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  while(Wire.available() < 14); 
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void read_MAG_DA5883_data(){
  Wire.beginTransmission(MAG_5883_addr);
  Wire.write(0x00); //start with register 3.
  Wire.endTransmission();
  Wire.requestFrom(MAG_5883_addr, 6);
  if (6 <= Wire.available()){
    MgX = Wire.read()|Wire.read() << 8;
    MgY = Wire.read()|Wire.read() << 8;
    MgZ = Wire.read()|Wire.read() << 8;
  }
}
