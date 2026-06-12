import time
import numpy as np
import serial
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

# Classe GPS avec configuration et lecture des données
class GPS:
    def __init__(self, port='/dev/ttyAMA0', baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port=self.port, baudrate=self.baudrate)
        self.initialized = False
        self.current_milli_time = lambda: int(round(time.time() * 1000))
                
    def attach(self):
        if self.initialized:
            return False

        self.ser.write(str.encode('AT+CGNSPWR=1' + '\r\n'))  # Power on the GPS antenna
        time.sleep(1)
        if not self.wait_for_reply('OK\r\n', 2000):
            print('Failed to power on GPS')
            return False

        self.ser.write(str.encode('AT+CGNSTST=1' + '\r\n'))  # Start GPS test mode
        if not self.wait_for_reply('OK\r\n', 2000):
            print('Failed to start GPS test mode')
            return False
        
        self.initialized = True
        return True

    def detach(self):
        self.ser.write(str.encode('AT+CGNSPWR=0' + '\r\n'))  # Power off the GPS antenna
        self.initialized = False

    def get_readings(self):
        while True:
            if self.ser.in_waiting:
                received_nmea = self.ser.readline().decode('ascii', errors='replace').strip()
                if received_nmea.startswith('$GPGGA'):
                    parts = received_nmea.split(',')
                    if len(parts) >= 9:
                        try:
                            latitude = float(parts[2]) / 100.0  # Latitude en degrés décimaux
                            longitude = float(parts[4]) / 100.0  # Longitude en degrés décimaux
                            return {'Latitude': latitude, 'Longitude': longitude}
                        except ValueError as e:
                            print(f"ValueError during conversion: {e}")
            else:
                time.sleep(0.1)  # Small delay to prevent busy-waiting
        return None

    def wait_for_reply(self, reply, timeout):
        start_time = self.current_milli_time()
        while (self.current_milli_time() - start_time) < timeout:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('ascii', errors='replace').strip()
                if line == reply.strip():
                    return True
        return False

# Classe pour le filtre de Kalman
class KalmanFilter:
    def __init__(self, initPos, initVel, posStdDev, accStdDev, currTime):
        self.X = np.array([[np.float64(initPos)], [np.float64(initVel)]])
        self.I = np.identity(2)
        self.P = np.identity(2)
        self.H = np.identity(2)
        self.Q = np.array([[accStdDev*accStdDev, 0], [0, accStdDev*accStdDev]])
        self.R = np.array([[posStdDev*posStdDev, 0], [0, posStdDev*posStdDev]])
        self.currentTime = currTime

    def predict(self, accThisAxis, timeNow):
        deltaT = timeNow - self.currentTime
        self.B = np.array([[0.5 * deltaT * deltaT], [deltaT]])
        self.A = np.array([[1.0, deltaT], [0.0, 1.0]])
        self.u = np.array([[accThisAxis]])
        self.X = np.add(np.matmul(self.A, self.X), np.matmul(self.B, self.u))  # priori estimate
        self.P = np.add(np.matmul(np.matmul(self.A, self.P), np.transpose(self.A)), self.Q)
        self.currentTime = timeNow

    def update(self, pos, velThisAxis, posError, velError):
        self.z = np.array([[pos], [velThisAxis]])
        if not posError:
            self.R[0, 0] = posError * posError
        if not velError:
            self.R[1, 1] = velError * velError
        y = np.subtract(self.z, self.X)  # residue
        s = np.add(self.P, self.R)
        try:
            sInverse = np.linalg.inv(s)
        except np.linalg.LinAlgError:
            print('matrix not invertible')
            return
        K = np.matmul(self.P, sInverse)  # Kalman Gain
        self.X = np.add(self.X, np.matmul(K, y))  # posteriori estimate
        self.P = np.matmul(np.subtract(self.I, K), self.P)
        return y, K

    def getPredictedPos(self):
        return self.X[0, 0]

    def getPredictedVel(self):
        return self.X[1, 0]

    def getUpdatedPos(self):
        return self.X[0, 0]

    def getUpdatedVel(self):
        return self.X[1, 0]

# Filtre passe-bas pour l'accéléromètre
class LowPassFilter:
    def __init__(self, alpha=0.5):
        self.alpha = alpha
        self.prev_value = None

    def apply(self, value):
        if self.prev_value is None:
            self.prev_value = value
        else:
            self.prev_value = self.alpha * value + (1 - self.alpha) * self.prev_value
        return self.prev_value

# Conversion de degrés décimaux à mètres
def deg_to_meters(lat, lon):
    R = 6378137.0  # Rayon moyen de la Terre en mètres
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)
    x = R * lon_rad * np.cos(lat_rad)
    y = R * lat_rad
    return x, y

def main():
    gps = GPS()
    if not gps.attach():
        print("Failed to initialize GPS module.")
        return

    # Paramètres pour le filtre de Kalman
    dt = 0.1  # Intervalle de temps
    initPos = 0
    initVel = 0
    posStdDev = 0.1
    accStdDev = 0.1
    currTime = time.time()

    # Initialisation des filtres de Kalman pour X et Y
    kfx = KalmanFilter(initPos, initVel, posStdDev, accStdDev, currTime)
    kfy = KalmanFilter(initPos, initVel, posStdDev, accStdDev, currTime)

    # Création d'une instance MPU9250
    mpu = MPU9250(
        address_ak=AK8963_ADDRESS,
        address_mpu_master=MPU9050_ADDRESS_68,  # In case the MPU9250 is connected to another I2C device
        address_mpu_slave=None,
        bus=1,
        gfs=GFS_1000,
        afs=AFS_8G,
        mfs=AK8963_BIT_16,
        mode=AK8963_MODE_C100HZ
    )
    mpu.configure()

    # Offsets manuels pour l'accéléromètre
    ax_offset, ay_offset, az_offset = -0.02780200195312499, 0.0239426269531251, -0.047374999999999945

    # Filtres passe-bas pour chaque axe
    lpf_x = LowPassFilter(alpha=0.3)
    lpf_y = LowPassFilter(alpha=0.3)
    lpf_z = LowPassFilter(alpha=0.3)

    prev_lat, prev_lon = None, None

    while True:
        data = gps.get_readings()
        if data:
            try:
                latitude = data.get('Latitude', None)
                longitude = data.get('Longitude', None)

                if None not in (latitude, longitude):
                    accel_data = mpu.readAccelerometerMaster()
                    print("Accelerometer raw data:", accel_data)

                    # Appliquer les offsets et convertir en m/s²
                    accel_data[0] = ((accel_data[0] - ax_offset) * 9.80665) - 0.2559
                    accel_data[1] = ((accel_data[1] - ay_offset) * 9.80665) - 0.2460
                    accel_data[2] = ((accel_data[2] - az_offset) * 9.80665) - 0.1268

                    # Appliquer le filtre passe-bas
                    accel_data[0] = lpf_x.apply(accel_data[0])
                    accel_data[1] = lpf_y.apply(accel_data[1])
                    accel_data[2] = lpf_z.apply(accel_data[2])
                    print("Accelerometer calibrated and filtered:", accel_data)

                    currTime = time.time()
                    dt = currTime - kfx.currentTime

                    # Prédiction des mesures de l'accéléromètre avec le filtre de Kalman
                    kfx.predict(accel_data[0], currTime)
                    kfy.predict(accel_data[1], currTime)

                    # Mise à jour du filtre de Kalman avec les données GPS
                    pos_x, pos_y = deg_to_meters(latitude, longitude)
                    y_x, K_x = kfx.update(pos_x, accel_data[0], posStdDev, accStdDev)
                    y_y, K_y = kfy.update(pos_y, accel_data[1], posStdDev, accStdDev)

                    # Affichage des résultats
                    print("Predicted Position (X):", kfx.getPredictedPos())
                    print("Predicted Velocity (X):", kfx.getPredictedVel())
                    print("Updated Position (X):", kfx.getUpdatedPos())
                    print("Updated Velocity (X):", kfx.getUpdatedVel())

                    print("Predicted Position (Y):", kfy.getPredictedPos())
                    print("Predicted Velocity (Y):", kfy.getPredictedVel())
                    print("Updated Position (Y):", kfy.getUpdatedPos())
                    print("Updated Velocity (Y):", kfy.getUpdatedVel())

            except Exception as e:
                print(f"Exception in main loop: {e}")

        time.sleep(0.1)  # Small delay to prevent busy-waiting

if __name__ == "__main__":
    main()
