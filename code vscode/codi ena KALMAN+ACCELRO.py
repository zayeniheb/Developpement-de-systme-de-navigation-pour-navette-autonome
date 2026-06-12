import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
import numpy as np 
from filterpy.kalman import KalmanFilter


def calibrate_accel(mpu, num_samples=1000):
    print("Calibrating accelerometer...")
    #Initialise une liste accel_offsets pour stocker les offsets d'accélération calibrés pour les axes x, y et z. Au début, tous les offsets sont définis à 0.
    accel_offsets = [0, 0, 0] 
    #•	Définit une liste axis_labels contenant les étiquettes des axes x, y et z, utilisées pour l'affichage.
    axis_labels = ['x', 'y', 'z']
    
    for axis in range(3):
        print(f"Orienter le capteur où l'axe {axis_labels[axis]} est contre la gravité.")
        input("Appuyez sur Entrée lorsque prêt...")
        
        accel_data = []
        for _ in range(num_samples):
            #Lit les données d'accélération pour l'axe spécifié (x, y ou z) et les ajoute à accel_data.
            accel_data.append(mpu.readAccelerometer(axis))
            time.sleep(0.01)
        
        accel_offset = np.mean(accel_data)#Calcule la moyenne des données d'accélération collectées.
        accel_offsets[axis] = accel_offset - 1 #: Soustrait 1 à l'offset moyen calculé 
        
        print("{axis_labels[axis]} axis: {accel_offsets[axis]}")
        
    print("calibrating accelerometre finish")
    print("accel offsets: ax_offset={},ay_offset={},az_offset={}",format(accel_offset))
    
    return accel_offsets


     

# Create an MPU9250 instance
mpu = MPU9250(
    address_ak=AK8963_ADDRESS,#The I2C address of the AK8963 magnetometer
    address_mpu_master=MPU9050_ADDRESS_68,  # The I2C address of the MPU9250 
    address_mpu_slave=None,#would be used if the MPU9250 is connected to another I2C device.
    bus=1,#The I2C bus number (commonly 1 for Raspberry Pi).
    gfs=GFS_1000,#Gyroscope full scale range (1000 degrees per second in this case).
    afs=AFS_8G,#Accelerometer full scale range (8G in this case).
    mfs=AK8963_BIT_16,#Magnetometer full scale range (16-bit resolution).
                    #Cela signifie que le capteur fournit des données de 16 bits pour chaque axe de mesure (X, Y, Z). Cette résolution détermine la précision des mesures.
    mode=AK8963_MODE_C100HZ)# magnétomètre mesure continuellement le champ magnétique a 100HZ 
                            


# Configure the MPU9250
mpu.configure()
ax_offset,ay_offset,az_offset=calibrate_accel(mpu)

def initialize_kalman_filter():
    kf = KalmanFilter(dim_x=3, dim_z=3)
    kf.x = np.array([0., 0., 0.])  # État initial
    kf.P *= 10.  # Matrice de covariance initiale
    kf.F = np.eye(3)  # Matrice de transition d'état
    kf.H = np.eye(3)  # Matrice de mesure
    kf.R = np.diag([0.5, 0.5, 0.5])  # Matrice de covariance de mesure
    kf.Q = np.diag([0.01, 0.01, 0.01])  # Matrice de covariance du bruit de processus
    return kf

kf = initialize_kalman_filter()

while True:
    # Read the accelerometer, gyroscope, and magnetometer values
    accel_data = mpu.readAccelerometer()

    print("Accelerometer not calibrated:", accel_data)
  
    accel_data[0]-=ax_offset
    accel_data[1]-=ay_offset
    accel_data[2]-=az_offset

    # Print the sensor values
    print("Accelerometer calibrated:", accel_data)
    kf.predict()
    kf.update([ accel_data[0],  accel_data[1],  accel_data[2]])
        
        # Obtenir les valeurs filtrées
    ax_filtered, ay_filtered, az_filtered = kf.x

    print(f"Accel X (Filtered): {ax_filtered:.4f} g\tAccel Y (Filtered): {ay_filtered:.4f} g\tAccel Z (Filtered): {az_filtered:.4f} g")
        
    time.sleep(1)
   

   

   
    