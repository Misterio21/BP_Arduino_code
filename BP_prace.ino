
//importování knihoven
#include <Wire.h>
#include <VL53L1X.h>
#include <NewPing.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <VL53L1X.h>

// přiřazení pinů pro jednotlivé senzory
#define RXgps 2
#define TXgps 3
#define pinTrigger 4
#define pinEcho 5
#define LED 7
#define TX 10
#define RX 11
#define maxRange 600

// inicializace senzorů a komunikace s Arduinem
VL53L1X sensor;
NewPing sonar(pinTrigger, pinEcho, maxRange);
TinyGPS gps;
SoftwareSerial swSerial(RXgps, TXgps);
SoftwareSerial bluetooth(TX, RX);

// inicializace potřebných proměnných
unsigned long startTime;
bool measuring = false;
bool speeding = true;
int range_beside_int_min = 1000; // Vysoká počáteční hodnotu pro minimální vzdálenost
int speed = 0; // Rychlost přibližujícího se objektu
unsigned long lastMeasurementTime;
unsigned int lastRangeBehind; 
unsigned int range_behind_int;
unsigned long currentTime;
bool LED_on = false;

void setup()
{
  //nasatvení monitorů
  bluetooth.begin(9600);
  swSerial.begin(9600);
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    bluetooth.println("*Nelze detekovat senzor");
    while (1);
  }
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);

  pinMode(LED, OUTPUT);
}

void loop()
{
  // vynulování proměnných pro nové získání GPS polohy
  bool novaData = false;
  unsigned long znaky;
  unsigned short slova, chyby;
  
  // kontrola signálu GPS
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (swSerial.available()) {
      // vytvoření proměnné pro uložení načtených dat z GPS
      char c = swSerial.read();
      // dekódování přijaté zprávy s kontrolou platných dat
      if (gps.encode(c)) {
        // pokud jsou přijatá nová data
        novaData = true;
      }
    }
  }
  //pouze pokud ke gps signál
  if (novaData) {
    //vynulování proměnných pro data ze signálu
    float zSirka, zDelka;
    unsigned long stariDat;
    gps.f_get_position(&zSirka, &zDelka, &stariDat);
    float sirka = (zSirka == TinyGPS::GPS_INVALID_F_ANGLE) ? 0.0 : zSirka;
    float delka = (zDelka == TinyGPS::GPS_INVALID_F_ANGLE) ? 0.0 : zDelka;
    float staridat = (stariDat == TinyGPS::GPS_INVALID_AGE) ? 0 : stariDat;
  range_behind_int = sensor.read();
  currentTime = millis();
  
  // kontrola zadní vzdálenosti
  if (!measuring && range_behind_int < 5000) {
    //objekt detekován
    LED_on = true;
    measuring = true;
    startTime = millis();
    lastRangeBehind = sensor.read();
    lastMeasurementTime = millis();
    delay(100);
  // zkoumání boční vzdálenosti
  } else if (measuring) {
    int range_beside_int = sonar.ping_cm();
    // Vypočet rychlosti
    unsigned long timeDelta = millis() - lastMeasurementTime;
    if (timeDelta > 0 && speeding) {
      speed = (float)(lastRangeBehind - range_behind_int) / timeDelta * 1000.0 / 100.0;
      speed = speed * 3.6;
      lastRangeBehind = range_behind_int;
      lastMeasurementTime = currentTime;
      speeding = false;
    }

    // Aktualizace minimální vzdálenosti
    if (range_beside_int < range_beside_int_min) {
      range_beside_int_min = range_beside_int;
    }

    // Odesílání dat pro upozornění uživatele
    bluetooth.println(String(range_behind_int) + "," + String(speed));
    
    if (currentTime - startTime > 8000) {
      // Odeslání minimální vzdálenosti
      bluetooth.println(String(zSirka) + "," + String(zDelka) +  "," + String(range_beside_int_min));
      measuring = false;
      range_beside_int_min = 1000; // Reset minimální vzdálenosti pro další měření
      speeding = true;
    }
  }
  else{
    //blikání během jízdy
    LED_on = !LED_on;
    digitalWrite(LED, LED_on);   
    bluetooth.println("-Volno,bezpecno");
  }
}
gps.stats(&znaky, &slova, &chyby);
  if (znaky == 0) {
    bluetooth.println("*Chyba pri prijmu dat z GPS");
  }
  delay(100);
}
