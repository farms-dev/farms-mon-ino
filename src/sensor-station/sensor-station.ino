#include <LoRa.h>
#include <OneWire.h>
#include <LowPower.h>

#define MOIST_SENSOR_PIN A0
#define MOIST_POWER_PIN 6
#define SOIL_TEMP_ID 3
#define SOIL_MOIST_ID 4

const long interval = 400000;

float soilTemperature;
float soilMoisture; 
long previousMillis = 0 - interval;
int counter = 1;

OneWire  ds(5);

float getSoilTemperature() {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float temperature = NULL;
  bool status = false;
  
  for (int a = 0; a < 2; a++) {
    if ( !ds.search(addr)) {
      Serial.println("  No more addresses.");
      ds.reset_search();
      delay(250);
    } else {
       status = true;
    }
  }

  if (status == true) {
    Serial.print("  ROM =");
    for( i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    }
    if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("  CRC is not valid!");
        return NULL;
    }
    Serial.println();
   
    // the first ROM byte indicates which chip
    switch (addr[0]) {
      case 0x10:
        Serial.println("  Chip = DS18S20"); // or old DS1820
        type_s = 1;
        break;
      case 0x28:
        Serial.println("  Chip = DS18B20");
        type_s = 0;
        break;
      case 0x22:
        Serial.println("  Chip = DS1822");
        type_s = 0;
        break;
      default:
        Serial.println("  Device is not a DS18x20 family device.");
        //return;
        break;
    } 
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);// start conversion, with parasite power on at the end
    
    delay(1000); // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.
    
    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE); // Read Scratchpad

    Serial.print("  Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");

    for ( i = 0; i < 9; i++) { // we need 9 bytes
      data[i] = ds.read();

      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    
    Serial.print(" CRC=");
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.println();
    // Convert the data to actual temperature, because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits, even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }

    temperature = (raw / 16.0);

    Serial.print(" raw temperature: ");
    Serial.println(raw);
    Serial.print(" soil temperature: ");
    Serial.println(temperature);

    return temperature;
  } else {
    Serial.println(" Failed to read temperature from Soil sensor!");
    return sqrt (-1); // NaN
  }
}

float getSoilMoisture() {
  float moistureRaw = NULL;
  float moisture = NULL;
  digitalWrite(MOIST_POWER_PIN, HIGH); // Powers up the moisture sensor for 0.1 seconds to prevent corrosion.
  moistureRaw = analogRead(MOIST_SENSOR_PIN);
  delay (100);
  digitalWrite(MOIST_POWER_PIN, LOW); // Powers down the moisture sensor

  Serial.print(" raw moisture: ");
  Serial.println(moistureRaw);

  if (moistureRaw == 0) {
    Serial.println("Failed to read moisture from Soil sensor!");
    return sqrt (-1); // NaN
  } else {
    // moisture = (int)moistureRaw*0.18; // The sensor is calibrated by multiplying by 0.102.
    moisture = moistureRaw*0.112; // The sensor is calibrated by multiplying by 0.102.
    
  Serial.print(" soil moisture: ");
  Serial.println(moisture);

    //delay (2000);
    return moisture;
  }
}

void setup() {
  pinMode(MOIST_POWER_PIN, OUTPUT);

  Serial.begin(9600);
  while (!Serial);
  
  if (!LoRa.begin(8681E5)) {
    Serial.println(" Starting LoRa failed!");
    while (1);
  }

  Serial.println(" LoRa started");

  LoRa.end();
}

void loop() {
    unsigned long currentMillis = millis();

    Serial.print(" init loop time: ");

    if (currentMillis - previousMillis >= interval) {

      if (!LoRa.begin(8681E5)) {
        Serial.println(" Starting LoRa failed!");
      } else {
        // ======= transfer =======
        Serial.print("Sending packet: ");
        Serial.println(counter);
  
        LoRa.beginPacket(); 
  
        LoRa.print("|¡|");
        LoRa.print("{");
        LoRa.print("\"stationId\":\"");
        LoRa.print("1"); //TODO: add variable
        LoRa.print("\",");
        LoRa.print("\"count\":\"");
        LoRa.print(counter);
        LoRa.print("\",");
        LoRa.print("\"type\":\"D\",");
        LoRa.print("\"list\": [");
  
        /* soilTemperature = getSoilTemperature();
        if (!isnan(soilTemperature)) {
          Serial.print("Soil temperature: ");
          Serial.print(soilTemperature);
          Serial.println(" C");
        }*/
  
        soilMoisture = getSoilMoisture();
        if (!isnan(soilMoisture)) {
        
          Serial.print("Soil moisture: ");
          Serial.print(soilMoisture);
          Serial.println(" %");
  
          LoRa.print("{");
          LoRa.print("\"sensorId\":\"");
          LoRa.print(SOIL_MOIST_ID);
          LoRa.print("\",");
          LoRa.print("\"value\":\"");
          LoRa.print(soilMoisture);
          LoRa.print("\"}");
        }
  
        LoRa.println("]}|!|");
        LoRa.endPacket();
        LoRa.end();
        previousMillis = currentMillis;
        counter++;
      }
    }

    delay(500);
    digitalWrite(LED_BUILTIN,LOW);
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
    delay(392);
}
