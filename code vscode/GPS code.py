import serial
import time
import pynmea2 #un format standard pour les données de navigation maritime et GPS (NMEA)

def parse_gps(data):#Déclare une fonction pour analyser les données GPS.
    if data.startswith('$GPGGA'):# Vérifie si les données commencent par '$GPGGA' (phrase NMEA pour les informations de position).
        msg = pynmea2.parse(data) #Analyse les données NMEA avec pynmea2
        #extraction de laltitude /logntitude/altitude
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude
        return latitude, longitude, altitude
    return None, None, None #Retourne None si les données ne sont pas un message GPGGA

def parse_velocity(data):
    #a phrase NMEA $GPVTG (Track Made Good and Ground Speed) fournit des informations sur la direction et la vitesse du mouvement
    if data.startswith('$GPVTG'):
        msg = pynmea2.parse(data)
        #extraire la vitesse en km/h
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
port = "/dev/ttyUSB0"  # Change to your serial port(par exemple com3)
baudrate = 9600
ser = serial.Serial(port, baudrate, timeout=1) #Ouvre la connexion série avec le port et le débit définis.

try:
    while True:
        line = ser.readline().decode('ascii')#convertit les octets lus depuis le port série en une chaîne de caractères en utilisant le jeu de caractères ASCII.
        latitude, longitude, altitude = parse_gps(line) #Appelle parse_gps pour extraire les données GPS.
        if latitude and longitude and altitude:
            print(f"Position: Latitude={latitude}, Longitude={longitude}, Altitude={altitude}m")
        
        vx, vy, vz = parse_velocity(line) ##Appelle parse_gps pour extraire les données de vitesse
        if vx is not None and vy is not None and vz is not None:
            print(f"Velocity: Vx={vx} m/s, Vy={vy} m/s, Vz={vz} m/s")
        
        time.sleep(1)


finally:
    ser.close()
