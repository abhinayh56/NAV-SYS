#include<Wire.h>
#define MAG_5883_addr 0x0D
int MgX, MgY, MgZ;
float t;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  TWBR = 12;     
  setup_MAG_DA5883_registers();
  t = micros();
}

void loop() {
  read_MAG_DA5883_data();
  Serial.print(MgX); Serial.print('\t'); Serial.print(MgY); Serial.print('\t'); Serial.print(MgZ); Serial.print('\t');
  Serial.println(micros()-t);
  t = micros();
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
