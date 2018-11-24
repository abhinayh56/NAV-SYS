#include<Wire.h>
const int MPU_addr=0x68; 
float AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float SampleFrequency = 250, dt = 1/SampleFrequency, loop_timer = 1000000*dt, t;
float GyXOff, GyYOff, GyZOff;
float acc_sensitivity=4096, gyro_sensitivity=65.5;
float p, q, r;
float phi_d_gyro, th_d_gyro, psi_d_gyro;
float phi_gyro, th_gyro, psi_gyro;
float AcX_P,AcY_P,AcZ_P;
float amx, amy, amz;
float phi_acc, th_acc;
float k1=0.95, k2=0.004;
int start;

void setup(){
  Serial.begin(115200);
  Wire.begin();
  TWBR = 12;     
  setup_mpu_6050_registers();
  calc_gyro_offset();
  /*GyXOff = -47.33;
  GyYOff = -21.50;
  GyZOff = -39.23;
*/
  t=micros();
}

void loop(){
  read_mpu_6050_data();
  LPF_acc();
  acc_angle_calc();
  gyro_angle_calc();
  gyro_drift_correction();
  print_angles();
  
  while(micros()-t < loop_timer){}
  t=micros();
}

void setup_mpu_6050_registers(){
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

void read_mpu_6050_data(){
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

void LPF_acc(){
  AcX = AcX*(1-k1) + k1*AcX_P;
  AcY = AcY*(1-k1) + k1*AcY_P;
  AcZ = AcZ*(1-k1) + k1*AcZ_P;
  AcX_P = AcX;
  AcY_P = AcY;
  AcZ_P = AcZ;
}

void acc_angle_calc(){
  amx = AcX/acc_sensitivity;
  amy = AcY/acc_sensitivity;
  amz = AcZ/acc_sensitivity;
  
  float g = sqrt(amx*amx + amy*amy + amz*amz);
  phi_acc = atan2(amy,amz);
  th_acc = -asin(amx/g);
}

void calc_gyro_offset(){
  float n=500;
  Serial.println("");
  Serial.println("Calliberating Gyroscope !");
  for(int i=0;i<n;i++){
    if(i % 100 == 0){
      Serial.print("- ");
    }
    read_mpu_6050_data();
    GyXOff=GyXOff+GyX;
    GyYOff=GyYOff+GyY;
    GyZOff=GyZOff+GyZ;
    delay(5); 
  }
  GyXOff=GyXOff/n;
  GyYOff=GyYOff/n;
  GyZOff=GyZOff/n;
  Serial.println("");
  Serial.println("Gyro Caliberation Complete!!");
  Serial.println("Offsets are:");
  Serial.print("GyXOff = "); Serial.println(GyXOff);
  Serial.print("GyYOff = "); Serial.println(GyYOff);
  Serial.print("GyZOff = "); Serial.println(GyZOff);
  Serial.print("\n");
}

void gyro_angle_calc(){
  GyX -= GyXOff;
  GyY -= GyYOff;
  GyZ -= GyZOff;
  p = GyX/gyro_sensitivity;
  q = GyY/gyro_sensitivity;
  r = GyZ/gyro_sensitivity;

  p = p*PI/180;
  q = q*PI/180;
  r = r*PI/180;
  
  phi_d_gyro =  p + (q*sin(phi_gyro) + r*cos(phi_gyro))*tan(th_gyro);
  th_d_gyro  =  q*cos(phi_gyro) - r*sin(phi_gyro);
  psi_d_gyro = (q*sin(phi_gyro) + r*cos(phi_gyro))/cos(th_gyro);
  
  phi_gyro = phi_gyro + phi_d_gyro*dt;
  th_gyro  = th_gyro  + th_d_gyro*dt;
  psi_gyro = psi_gyro + psi_d_gyro*dt;

  if(start==0){
    start = 1;
    phi_gyro = phi_acc;
    th_gyro  = th_acc;
  }
}

void gyro_drift_correction(){
  phi_gyro = phi_gyro + k2*(phi_acc - phi_gyro);
  th_gyro = th_gyro + k2*(th_acc - th_gyro);
}

void print_angles(){
  Serial.print(phi_gyro*180/PI,3); Serial.print("\t");
  Serial.print(th_gyro*180/PI,3); Serial.print("\t");
  Serial.println(psi_gyro*180/PI,3);
}
