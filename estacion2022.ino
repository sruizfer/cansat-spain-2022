//Programacion estacion principal

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

#include <SPI.h>
#include <LoRa.h>
#include <RP2040_SD.h>

//Archivo de la sd
File myFile;
#define fileName  "datos.csv"

String cadena;
String prov;
String datos[15];
String all;
int contador;

void setup() {
  Serial.begin(9600);

  if (!LoRa.begin(869.8E6)) {
    Serial.println("Error al iniciar lora");
    while (1);
  }

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
}

void loop() {
  cadena = "";
  prov = "";
  contador = 0;
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    while (LoRa.available()) {
      cadena = cadena + (char)LoRa.read();
    }
    for (int h = 0; h < cadena.length(); h++) {
      if (cadena[h] != ',') {
        prov = prov + cadena[h];
      } else if (cadena[h] == ',') {
        datos[contador] = prov;
        prov = "";
        contador ++;
      }
    }
  }
  if (cadena != "") {
    all += datos[12];
    all += ",";
    all += datos[10];
    all += ",";
    all += datos[11];
    Serial.println(all);
    myFile = SD.open(fileName, FILE_WRITE);
    if (myFile) {
      myFile.println(cadena);
      myFile.close();
    }
    all = "";
  }
}
