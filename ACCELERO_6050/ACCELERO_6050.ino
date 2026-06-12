#include <Wire.h>          // Inclut la bibliothèque Wire pour la communication I2C
#include <MPU6050.h>       // Inclut la bibliothèque MPU6050 pour interagir avec le capteur MPU6050

MPU6050 mpu;               // Crée un objet MPU6050 pour gérer les interactions avec le capteur

float gx_offset, gy_offset, gz_offset;  // Variables pour stocker les offsets du gyroscope
float ax_offset, ay_offset, az_offset;  // Variables pour stocker les offsets de l'accéléromètre
int num_samples = 1000;    // Nombre d'échantillons à utiliser pour la calibration

void setup() {
  Serial.begin(9600);      // Initialise la communication série à 9600 bauds pour la sortie des données
  Wire.begin();            // Initialise la communication I2C
  mpu.initialize();        // Initialise le capteur MPU6050

  // Vérifie la connexion avec le capteur MPU6050
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");  // Si la connexion échoue, affiche un message
    while (1);            // Bloque le programme ici si la connexion échoue
  }

  Serial.println("MPU6050 initialized!");  // Si la connexion réussit, affiche un message

  // Calibrer le gyroscope et l'accéléromètre
  calibrateGyro();         // Appelle la fonction pour calibrer le gyroscope
  calibrateAccel();        // Appelle la fonction pour calibrer l'accéléromètre
}

void loop() {
  // Déclare des variables pour stocker les valeurs brutes de l'accéléromètre et du gyroscope
  int16_t ax, ay, az, gx, gy, gz;
  
  // Lit les données de mouvement (accéléromètre et gyroscope)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Applique les offsets de calibration pour le gyroscope
  gx -= gx_offset;         // Soustrait l'offset calibré pour gx
  gy -= gy_offset;         // Soustrait l'offset calibré pour gy
  gz -= gz_offset;         // Soustrait l'offset calibré pour gz

  // Applique les offsets de calibration pour l'accéléromètre
  ax -= ax_offset;         // Soustrait l'offset calibré pour ax
  ay -= ay_offset;         // Soustrait l'offset calibré pour ay
  az -= az_offset;         // Soustrait l'offset calibré pour az

  // Affiche les valeurs calibrées de l'accéléromètre
  Serial.print("Accelerometer calibrated: ax = "); Serial.print(ax);
  Serial.print(", ay = "); Serial.print(ay);
  Serial.print(", az = "); Serial.println(az);

  // Affiche les valeurs calibrées du gyroscope
  Serial.print("Gyroscope calibrated: gx = "); Serial.print(gx);
  Serial.print(", gy = "); Serial.print(gy);
  Serial.print(", gz = "); Serial.println(gz);

  delay(1000);             // Attends 1 seconde avant de relire les valeurs
}

void calibrateGyro() {
  Serial.println("Calibrating gyroscope...");  // Affiche un message indiquant le début de la calibration du gyroscope
  
  // Sommes pour accumuler les valeurs de l'axe du gyroscope
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  
  // Effectue la lecture et l'accumulation des données sur un certain nombre d'échantillons
  for (int i = 0; i < num_samples; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);  // Lit les valeurs brutes du gyroscope

    gx_sum += gx;          // Accumule les données pour gx
    gy_sum += gy;          // Accumule les données pour gy
    gz_sum += gz;          // Accumule les données pour gz

    delay(10);             // Attend 10ms entre chaque lecture
  }

  // Calcule les offsets en prenant la moyenne des valeurs accumulées
  gx_offset = gx_sum / num_samples;
  gy_offset = gy_sum / num_samples;
  gz_offset = gz_sum / num_samples;

  // Affiche les offsets du gyroscope
  Serial.print("Gyroscope offsets: gx_offset = "); Serial.print(gx_offset);
  Serial.print(", gy_offset = "); Serial.print(gy_offset);
  Serial.print(", gz_offset = "); Serial.println(gz_offset);
}

void calibrateAccel() {
  Serial.println("Calibrating accelerometer...");  // Affiche un message indiquant le début de la calibration de l'accéléromètre
  
  // Sommes pour accumuler les valeurs de l'axe de l'accéléromètre
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  
  // Effectue la lecture et l'accumulation des données sur un certain nombre d'échantillons
  for (int i = 0; i < num_samples; i++) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);  // Lit les valeurs brutes de l'accéléromètre

    ax_sum += ax;          // Accumule les données pour ax
    ay_sum += ay;          // Accumule les données pour ay
    az_sum += az;          // Accumule les données pour az

    delay(10);             // Attend 10ms entre chaque lecture
  }

  // Calcule les offsets de l'accéléromètre, en supposant que l'axe z est aligné avec la gravité
  ax_offset = ax_sum / num_samples;
  ay_offset = ay_sum / num_samples;
  az_offset = (az_sum / num_samples) - 16384;  // 1g (~9.81 m/s²) correspond à environ 16384 dans les données brutes

  // Affiche les offsets de l'accéléromètre
  Serial.print("Accelerometer offsets: ax_offset = "); Serial.print(ax_offset);
  Serial.print(", ay_offset = "); Serial.print(ay_offset);
  Serial.print(", az_offset = "); Serial.println(az_offset);
}
