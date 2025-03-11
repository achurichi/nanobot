from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

mpu = MPU9250(
    address_ak=AK8963_ADDRESS, 
    address_mpu_master=MPU9050_ADDRESS_68, # Master has 0x68 Address
    address_mpu_slave=None, 
    bus=1, 
    gfs=GFS_250, 
    afs=AFS_4G, 
    mfs=AK8963_BIT_16, 
    mode=AK8963_MODE_C100HZ)

mpu.configure() # Apply the settings to the registers.

mpu.calibrateAK8963() # Calibrate sensors

mpu.configure() # The calibration function resets the sensors, so you need to reconfigure them

mbias_tesla = [value / 1e6 for value in mpu.mbias]

print(f"Magnetometer bias in Tesla units:")
print(f"x: {mbias_tesla[0]}")
print(f"y: {mbias_tesla[1]}")
print(f"z: {mbias_tesla[2]}")