The folder includes 4 arduino codes:

..........................................................................................................................................
:        File_Name                                       Description							                 :
:........................................................................................................................................:
:  1_i2c_device_scanner       ----->    scans I2C address of the sensor
:  2_read_sensors             ----->    gives raw accelerometer, gyroscope and temperature data				                 :
:  3_acc_angle                ----->    gives Euler angle(phi,th not psi) using ZYX(=psi-th-phi) sequence using only accelerometer       :
:  4_gyro_angle  	      ----->    gives Euler angle(phi,th,psi) using ZYX(=psi-th-phi) sequence using only gyroscope               :
:  5_gyro_drift_correction    ----->    gives Euler angle(phi,th,psi) using ZYX(=psi-th-phi) sequence of gyroscope after correction done :
:					by accelerometer measurements to compensate drift                 			         :
:........................................................................................................................................:

NOTE  : This program calculates orientation using Euler angle. So it will not work near pitch angle +-90 degrees due to simgularity 
	in representation of Euler angle itself (devision by cos(th)). If pitch angle is +-90 degree then yaw axis (Z axis of inertial 
	frame) and roll axis (X axis of body frame) concides.
