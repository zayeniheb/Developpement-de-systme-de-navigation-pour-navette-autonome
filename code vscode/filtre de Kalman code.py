import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
import numpy as np 
import serial
import time
import pynmea2
# //////////code de GPS//////////////
def parse_gps(data):
    if data.startswith('$GPGGA'):
        msg = pynmea2.parse(data)
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude
        return latitude, longitude, altitude
    return None, None, None

def parse_velocity(data):
    if data.startswith('$GPVTG'):
        msg = pynmea2.parse(data)
        speed_kph = msg.spd_over_grnd_kmph
        return speed_kph / 3.6  # Convert from km/h to m/s
    elif data.startswith('$GPXYZ'):  # Assuming $GPXYZ is the custom message for 3D velocity
        # Parse the custom 3D velocity message
        parts = data.split(',')
        vx = float(parts[1])  # Velocity in X axis (m/s)
        vy = float(parts[2])  # Velocity in Y axis (m/s)
        vz = float(parts[3])  # Velocity in Z axis (m/s)
        return vx, vy, vz
    return None, None, None

# Configure the serial port
port = "/dev/ttyUSB0"  # Change to your serial port
baudrate = 9600
ser = serial.Serial(port, baudrate, timeout=1)



# //////////code de MPU9250//////////////

def calibrate_gyro(mpu ,num_samples=1000):
 print("calibrating gyroscope")
 gyro_data=[]
 for _ in  range(num_samples):
    gyro_data.append(mpu.readyGyroscopeMaster())
    time.sleep(0.01)
    gyro_data=np.array(gyro_data)
    gx_offset,gy_offset,gz_offset=np.mean(gyro_data,axis=(0))
    print("calibrating gyroscope finish")
    print("gyroscopeo offsets: gx_offset={},gy_offset={},gz_offset={}",format(gx_offset,gy_offset,gz_offset))
 return gx_offset,gy_offset,gz_offset




def calibrate_accel(mpu, num_samples=1000):
    print("Calibrating accelerometer...")
    accel_offsets = [0, 0, 0]
    axis_labels = ['x', 'y', 'z']
    
    for axis in range(3):
        print(f"Orienter le capteur où l'axe {axis_labels[axis]} est contre la gravité.")
        input("Appuyez sur Entrée lorsque prêt...")
        
        accel_data = []
        for _ in range(num_samples):
            accel_data.append(mpu.readAccelerometer(axis))
            time.sleep(0.01)
        
        accel_offset = np.mean(accel_data)
        accel_offsets[axis] = accel_offset - 1
        
        print("{axis_labels[axis]} axis: {accel_offsets[axis]}")
        
    print("calibrating accelerometre finish")
    print("accel offsets: ax_offset={},ay_offset={},az_offset={}",format(accel_offset))
    
    return accel_offsets


     

# Create an MPU9250 instance
mpu = MPU9250(
    address_ak=AK8963_ADDRESS,
    address_mpu_master=MPU9050_ADDRESS_68,  # In case the MPU9250 is connected to another I2C device
    address_mpu_slave=None,
    bus=1,
    gfs=GFS_1000,
    afs=AFS_8G,
    mfs=AK8963_BIT_16,
    mode=AK8963_MODE_C100HZ)

# Configure the MPU9250
mpu.configure()
gx_offset,gy_offset,gz_offset=calibrate_gyro(mpu)
ax_offset,ay_offset,az_offset=calibrate_accel(mpu)



# //////////code de foltre de kalmaaan //////////////

class KalmanFilter_x:
    def __init__(self, F, B, H, Q, R, x0, P0):
        self.F = F  # Matrice de transition d'état
        self.B = B  # Matrice de contrôle
        self.H = H  # Matrice d'observation
        self.Q = Q  # Matrice de covariance du bruit de processus
        self.R = R  # Matrice de covariance du bruit de mesure
        self.x0 = x0  # État du système (position et vitesse) sur axe x
        self.P0 = P0  # Matrice de covariance de l'état sur axe x
        
        
    def predict(self, u0):
        # Prédiction de l'état sur les trois axes
        self.x0 = np.dot(self.F, self.x0) + np.dot(self.B, u0)
        
        
        self.P0 = np.dot(np.dot(self.F, self.P0), self.F.T) + self.Q
        
        
        return self.x0
    
    def update(self, z1):
        # Calcul du gain de Kalman
        S = np.dot(np.dot(self.H, self.P0), self.H.T) + self.R
        K = np.dot(np.dot(self.P0, self.H.T), np.linalg.inv(S))
        
        # Calcul de l'innovation sur les trois axes
        y1 = z1 - np.dot(self.H, self.x0)
       
        
        # Mise à jour de l'état estimé sur les trois axes
        self.x0 = self.x0 + np.dot(K, y1)
        
        
        I = np.eye(self.P0.shape[0])  # Matrice identité de même dimension que P0
        self.P0 = np.dot(np.dot(I - np.dot(K, self.H), self.P0), (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)
       
       
        return self.x0
class KalmanFilter_y:
    def __init__(self, F, B, H, Q, R, x1, P1):
        self.F = F  # Matrice de transition d'état
        self.B = B  # Matrice de contrôle
        self.H = H  # Matrice d'observation
        self.Q = Q  # Matrice de covariance du bruit de processus
        self.R = R
        self.x1 = x1  # État du système (position et vitesse) sur axe y
        self.P1 = P1  # Matrice de covariance de l'état sur axe y
        
        
    def predict(self, u1):
        # Prédiction de l'état sur l'axe y
        self.x1 = np.dot(self.F, self.x1) + np.dot(self.B, u1)
        
        # Prédiction de la covariance sur l'axe y
        self.P1 = np.dot(np.dot(self.F, self.P1), self.F.T) + self.Q
        
        
        return  self.x1
    
    def update(self,  z2 ):
        # Calcul du covarinace de l'innovation 
        S2 = np.dot(np.dot(self.H, self.P1), self.H.T) + self.R
       # Calcul du gain de Kalman
        K2 = np.dot(np.dot(self.P1, self.H.T), np.linalg.inv(S2))
        
        # Calcul de l'innovation sur l'axe y
        y2 = z2 - np.dot(self.H, self.x1)
       
        
        # Mise à jour de l'état estimé sur l'ace y
        self.x1 = self.x1 + np.dot(K2, y2)
        
        
        I = np.eye(self.P1.shape[0])  # Matrice identité de même dimension que P1
        # Mise à jour de la covariance  estimé sur l'ace y
        self.P1 = np.dot(np.dot(I - np.dot(K2, self.H), self.P1), (I - np.dot(K2, self.H)).T) + np.dot(np.dot(K2, self.R), K2.T)
       
       
        return self.x1
class KalmanFilter_Z:
    def __init__(self, F, B, H, Q, R, x2, P2):
        self.F = F  # Matrice de transition d'état
        self.B = B  # Matrice de contrôle
        self.H = H  # Matrice d'observation
        self.Q = Q  # Matrice de covariance du bruit de processus
        self.R = R  # Matrice de covariance du bruit de mesure
        self.x2 = x2  # État du système (position et vitesse) sur axe z
        self.P2 = P2  # Matrice de covariance de l'état sur axe z
        
    def predict(self, u2):
        # Prédiction de l'état sur l'axe z
        self.x2 = np.dot(self.F, self.x2) + np.dot(self.B, u2)
        
         # Prédiction de la covariance sur l'axe z
        self.P2 = np.dot(np.dot(self.F, self.P2), self.F.T) + self.Q
        
        
        return  self.x2
    
    def update(self, z3):
        # Calcul du covariance d l'innovation 
        S3 = np.dot(np.dot(self.H, self.P2), self.H.T) + self.R
        # Calcul du gain de Kalman
        K3= np.dot(np.dot(self.P2, self.H.T), np.linalg.inv(S3))
        
        # Calcul de l'innovation sur l'axe z
        y3 = z3 - np.dot(self.H, self.x2)
       
        
        # Mise à jour de l'état estimé sur l'axe z
        self.x2 = self.x2 + np.dot(K3, y3)
        
        
        I = np.eye(self.P2.shape[0])  # Matrice identité de même dimension que P2
        # Mise à jour de la covariance  sur l'axe z
        self.P2 = np.dot(np.dot(I - np.dot(K3, self.H), self.P2), (I - np.dot(K3, self.H)).T) + np.dot(np.dot(K3, self.R), K3.T)
       
       
        return self.x2    

# Paramètres du filtre de Kalman
F = np.array([[1, 1], [0, 1]])
B = np.array([[0.5], [1]])
H = np.array([[1, 0]])
Q = np.array([[1, 0], [0, 1]])
R = np.array([[1]])
x0 = np.array([[0], [1]])
P0 = np.array([[1, 0], [0, 1]])
x1 = np.array([[0], [1]])
P1 = np.array([[1, 0], [0, 1]])
x2 = np.array([[0], [1]])
P2 = np.array([[1, 0], [0, 1]])

# Création des instances de KalmanFilter
kfx = KalmanFilter_x(F, B, H, Q, R, x0, P0, )#inctance sur l'axe x 
kfy = KalmanFilter_y(F, B, H, Q, R,  x1, P1)#inctance sur l'axe y 
kfz = KalmanFilter_Z(F, B, H, Q, R, x2, P2)#inctance sur l'axe z 








# //////////fusion des données//////////////
while True:
    
    line = ser.readline().decode('ascii', errors='replace')
    latitude, longitude, altitude = parse_gps(line)
    if latitude and longitude and altitude:
          print(f"Position: Latitude={latitude}, Longitude={longitude}, Altitude={altitude}m")
        
    vx, vy, vz = parse_velocity(line)
    if vx is not None and vy is not None and vz is not None:
            print(f"Velocity: Vx={vx} m/s, Vy={vy} m/s, Vz={vz} m/s")
        
    time.sleep(1)
    
    # Read the accelerometer, gyroscope, and magnetometer values
    accel_data = mpu.readAccelerometerMaster()
    gyro_data = mpu.readGyroscopeMaster()
    mag_data = mpu.readMagnetometerMaster()
    
    print("Accelerometer not calibrated:", accel_data)
    print("Gyroscope not calbrated:", gyro_data)
    
    gyro_data[0]-=gx_offset
    gyro_data[1]-=gy_offset
    gyro_data[2]-=gz_offset
    accel_data[0]-=ax_offset
    accel_data[1]-=ay_offset
    accel_data[2]-=az_offset
     # Print the sensor values
    print("Accelerometer calibrated:", accel_data)
    print("Gyroscope calbrated:", gyro_data)
    print("Magnetometer:", mag_data)
    
    
    u0 =accel_data[0]
    u1 =accel_data[1]
    u2 =accel_data[2]
    z1 = [latitude . vx ]  # Vecteur de mesure (mesure de GPS) sur axe x
    z2 = [longitude . vy ] # Vecteur de mesure (mesure de GPS) sur axe y
    z3 = [altitude . vz ] # Vecteur de mesure (mesure de GPS) sur axe z

    
    # Prédiction et mise à jour sur les trois axes
    predicted_state_x = kfx.predict(u0)
    print("État prédit sur axe x :\n", predicted_state_x)
    predicted_state_y = kfy.predict( u1)
    print("État prédit sur axe y:\n", predicted_state_y)
    predicted_state_z = kfz.predict( u2)
    print("État prédit sur axe z :\n", predicted_state_z)

    updated_state_x = kfx.update(z1)
    print("État mis à jour sur axe x:\n", updated_state_x)
    updated_state_y = kfy.update( z2 )
    print("État mis à jour sur axe y:\n", updated_state_y)
    updated_state_z = kfz.update( z3)
    print("État mis à jour sur axe z:\n", updated_state_z)


    # Wait for 1 second before the next reading
    time.sleep(1)
    
