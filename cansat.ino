//PROGRAMACION DEL CANSAT ARGONAUTEX2022
//en la linea 687 del archivo lora.cpp hay que poner el mismo numero en _implicitheadermode en el sender y receiver
//https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json

//Configuarcion de pines de la sd
#if !defined(ARDUINO_ARCH_RP2040)
error For RP2040 only
#endif

#if defined(ARDUINO_ARCH_MBED)

#define PIN_SD_MOSI       PIN_SPI_MOSI
#define PIN_SD_MISO       PIN_SPI_MISO
#define PIN_SD_SCK        PIN_SPI_SCK
#define PIN_SD_SS         PIN_SPI_SS

#else

#define PIN_SD_MOSI       PIN_SPI0_MOSI
#define PIN_SD_MISO       PIN_SPI0_MISO
#define PIN_SD_SCK        PIN_SPI0_SCK
#define PIN_SD_SS         PIN_SPI0_SS

#endif

#define _RP2040_SD_LOGLEVEL_       4

//Declaracion de librerias
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BME280.h>
#include "Adafruit_CCS811.h"
#include <Melopero_AMG8833.h>
#include <Adafruit_GPS.h>
#include <LSM6.h>
#include <RP2040_SD.h>

//Archivo de la sd
File myFile;
#define fileName  "datos.csv"

//Configuracion del GPS
#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false

//Declaracion de variables
long lastSendTime = 0;
int interval = 0;
char HWMessage[59];
uint8_t HWMessageLen;

//Instanciar objectos
Adafruit_BME280 bme;
Adafruit_CCS811 ccs;
Melopero_AMG8833 camara;
LSM6 ace;

//Declaracion para el sensor de polvo
int dustPin = 26;
int ledPin = 2;

//Pines para el led y el zumbador
int led = 6;
int zum = 22;

void setup() {
  //Declaramos el modo de los pines
  pinMode (led, OUTPUT);
  pinMode (zum, OUTPUT);

  //Iniciamos el serial
  Serial.begin(9600);

  //Iniciamos la sd
  if (!SD.begin(PIN_SD_SS))
  {
    Serial.println("Error al iniciar la sd");

    return;
  }
  myFile = SD.open(fileName, FILE_WRITE);

  if (myFile) {
    myFile.println("tiempo,temperatura,presion,humedad,altitud,co2,ace_x,ace_y,ace_z,pm10,latitud,longitud,altitud,induccion");
    myFile.close();
  }

  //Declaramos los pines del i2c
  Wire.setSDA(0);
  Wire.setSCL(1);

  //Iniciamos el bme280
  if (!bme.begin()) {
    while (1) delay(10);
  }

  //Iniciamos el ccs811
  if (!ccs.begin()) {
    while (1);
  }
  while (!ccs.available());
  delay(500);

  //Iniciamos la camara termica
  camara.initI2C();
  int statusCode = camara.resetFlagsAndSettings();
  statusCode = camara.setFPSMode(FPS_MODE::FPS_10);

  //Iniciamos el acelerometro
  if (!ace.init()) {
    while (1);
  }
  ace.enableDefault();

  // Sensor de polvo
  pinMode(ledPin, OUTPUT);

  //Iniciamos el GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);

  if (!LoRa.begin(869.8E6)) {
    Serial.println("Error al iniciar LoRa");
    while (1);
  }

  //Declaramos el pin del sensor de electromagnetismo
  pinMode(28, INPUT);

}

void loop() {
  // Leemos el GPS
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    //Serial.print(GPS.lastNMEA());
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  if (millis() - lastSendTime > interval) {
    digitalWrite (led, HIGH);
    digitalWrite (zum, HIGH);
    guardar_sd(crear_cadena());
    enviar_lora(crear_cadena());
    Serial.println(crear_cadena());
    lastSendTime = millis();
    interval = 1000;
  }
  digitalWrite(led, LOW);
  digitalWrite(zum, LOW);
}

//Funcion para leer los datos del bme280
String datos_bme() {
  float temperatura = bme.readTemperature();
  float presion = bme.readPressure();
  float humedad = bme.readHumidity();
  float altitud_bme = bme.readAltitude(1013.25);
  String valores_bme = String(temperatura);
  valores_bme += ",";
  valores_bme += String(presion);
  valores_bme += ",";
  valores_bme += String(humedad);
  valores_bme += ",";
  valores_bme += String(altitud_bme);
  return valores_bme;
}

//Funcion para leer los datos del ccs811
String datos_ccs() {
  String valores_ccs;
  valores_ccs = "0";
  if (ccs.available()) {
    if (!ccs.readData()) {
      float dioxido_carbono = ccs.geteCO2();
      valores_ccs = String(dioxido_carbono);
    }
  }
  return valores_ccs;
}

//Funcion para leer los datos del acelerometro
String datos_acel() {
  ace.read();
  char salida[50];
  snprintf(salida, sizeof(salida), "%d,%d,%d",
           ace.a.x, ace.a.y, ace.a.z);
  return salida;
}

//Funcion para leer los datos del sensor de polvo
String datos_polvo() {
  float voltsMeasured = 0;
  float calcVoltage = 0;
  float dustDensity = 0;
  String valores_polvo;
  digitalWrite(ledPin, LOW);
  delayMicroseconds(280);
  voltsMeasured = analogRead(dustPin);
  delayMicroseconds(40);
  digitalWrite(ledPin, HIGH);
  delayMicroseconds(9680);
  calcVoltage = voltsMeasured * (5.0 / 1024.0);
  dustDensity = 0.17 * calcVoltage - 0.1;
  valores_polvo = String(dustDensity);
  return valores_polvo;
}

//Funcion para leer los datos del GPS
String datos_GPS() {
  String valores_GPS;
  float latitud = GPS.latitude;
  float longitud = GPS.longitude;
  float altitud = GPS.altitude;

  valores_GPS = String(latitud);
  valores_GPS += ",";
  valores_GPS += String(longitud);
  valores_GPS += ",";
  valores_GPS += String(altitud);

  return valores_GPS;
}

//Funcion para leer los datos del sensor de electromagnetismo
String datos_electromagnetismo() {
  float radio = 0.03f;
  float ancho = 0.025f;
  float superficie = M_PI * 0.0009f;
  float resi = 1.4f;
  float mu = 4 * M_PI * pow(10, (-7));
  int N = 22;

  float datos_electr = analogRead(28);
  float resultados_electr = (31.33f * mu * N * N * (((radio * radio) * datos_electr) / (((8 * radio) + (11 * ancho)) * superficie * resi)));
  return String (resultados_electr);
}

//Funcion para leer los datos de la camara termica
String datos_camara() {
  String valores_camara;

  int statusCode = camara.updateThermistorTemperature();
  statusCode = camara.updatePixelMatrix();
  for (int x = 0; x < 8; x++) {
    for (int y = 0; y < 8; y++) {
      valores_camara += camara.pixelMatrix[y][x];
      valores_camara += ",";

    }
  }
  return String(valores_camara);
}

//Funcion para guardar los datos en la sd
void guardar_sd(String cadena) {
  myFile = SD.open(fileName, FILE_WRITE);
  if (myFile) {
    cadena += datos_camara();
    myFile.println(cadena);
    myFile.close();
  }
}

//Funcion para enviar por lora
void enviar_lora(String cadena) {
  LoRa_txMode();
  LoRa.beginPacket();
  LoRa.print(cadena);
  LoRa.endPacket(true);

}


void LoRa_txMode() {
  LoRa.idle();
  LoRa.disableInvertIQ();
}

String crear_cadena() {
  String cadena;
  cadena += millis();
  cadena += ",";
  cadena += datos_bme();
  cadena += ",";
  cadena += datos_ccs();
  cadena += ",";
  cadena += datos_acel();
  cadena += ",";
  cadena += datos_polvo();
  cadena += ",";
  cadena += datos_GPS();
  cadena += ",";
  cadena += datos_electromagnetismo();
  return cadena;
}
