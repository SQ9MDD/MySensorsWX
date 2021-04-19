// Copyright (c) 2021 SQ9MDD Rysiek Labus
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <Arduino.h>

// Wiring BME280 as I2C
// BME - ARDUINO
// VCC - 3.3V
// GND - GND
// SCL - A5
// SDA - A4

#define CHILD_ID_T            1                                                         // temp and rh sensor
#define CHILD_ID_H            2                                                         // temp and rh sensor
#define CHILD_ID_BARO         3                                                         // baro sensor
#define CHILD_ID_AI1          4                                                         // voltage sensor
#define BATTERY_SENSE_PIN     A0                                                        // voltage mesure pin

#include <MySensors.h>                                                                  // biblioteka mysensors
#include <Wire.h>                                                                       //
#include <Adafruit_Sensor.h>                                                            //
#include <Adafruit_BME280.h>                                                            //

Adafruit_BME280 bme;                                                                    //

unsigned long SLEEP_TIME = 300000;                                                      // sleep time between reads and data send (seconds * 1000 milliseconds)
float weight = 5.0;                                                                     // scale, filter for smoothing measurements
float avg_mesure = 0.0;                                                                 // variable for average value of the last measurements
float sensorValue = 0.0;                                                                //

MyMessage msgTemp(CHILD_ID_T, V_TEMP);                                                  //
MyMessage msgHum(CHILD_ID_H, V_HUM);                                                    //
MyMessage msgBaro(CHILD_ID_BARO, V_PRESSURE);                                           //
MyMessage msgAI1(CHILD_ID_AI1, V_VOLTAGE);                                              //

void send_temp(){                                                                       //
  float temperature = 0.0;                                                              //
  temperature = bme.readTemperature(); 
  if(temperature <= -44.1){
    asm volatile ("  jmp 0");                                                           // if bme readings falls reset device
  }                                                                                     //
  send(msgTemp.set((float)temperature, 1));                                             //
}

void send_hum(){                                                                        //
  float humidity = 0;                                                                   //
  humidity = bme.readHumidity();                                                        //
  send(msgHum.set((float)humidity, 1));                                                 //
}

void send_baro(){                                                                       //
  float baro = 0;                                                                       //
  float hPa_offset = float(ABOVE_SEA_LVL) * 0.10933;                                    //
  baro = (bme.readPressure() / 100.0F) + hPa_offset;                                    //
  send(msgBaro.set((float)baro, 1));                                                    //
}

void presentation(){                                                                    //
  char etykieta[] = "       ";                                                          //
  int addr = MY_NODE_ID;                                                                //
  sendSketchInfo("AS-300R-THB-Bat", "1.1");                                             //
  sprintf(etykieta,"R%02u.AI1",addr);  present(CHILD_ID_T, S_TEMP, etykieta);           //
  sprintf(etykieta,"R%02u.AI2",addr);  present(CHILD_ID_H, S_HUM, etykieta);            //
  sprintf(etykieta,"R%02u.AI3",addr);  present(CHILD_ID_BARO, S_BARO, etykieta);        //
  sprintf(etykieta,"R%02u.AI4",addr);  present(CHILD_ID_AI1, S_MULTIMETER, etykieta);   //
}

void setup(){                                                                           //
  // test for sensor
  if (!bme.begin(0x76)) {                                                               //
    Serial.println("Could not find a valid BME280 sensor, check wiring!");              //
    while (1);                                                                          //
  }  
  analogReference(INTERNAL);                                                            //
  pinMode(BATTERY_SENSE_PIN,INPUT);                                                     //  
    #ifdef INTERNAL_VOLT_MESURE
        avg_mesure = hwCPUVoltage() / 1000.0;
    #else
        avg_mesure = analogRead(BATTERY_SENSE_PIN) * 0.00502926;
    #endif
  send_temp();                                                                          //
  send_hum();                                                                           //
  send_baro();                                                                          //
  sleep(10000);                                                                         //
}

void loop(){                                                                            //
    #ifdef INTERNAL_VOLT_MESURE
        sensorValue = hwCPUVoltage() / 1000.0;
    #else
        sensorValue = analogRead(BATTERY_SENSE_PIN) * 0.00502926;
    #endif
    // smoothing readings 
    avg_mesure = (avg_mesure*(weight-1) + sensorValue) / weight;
    
    float bat_low = 3.4;                                                        // minimum effective voltage for 18650 cell
    float bat_max = 4.2;                                                        // maximum effective voltage for 18650 cell
    // calculate battery percent within range
    // 3,4V - 0%
    // 4,2V - 100%
    float batteryPcnt = 100.0/(bat_max-bat_low) * float(avg_mesure-bat_low);
    batteryPcnt = constrain(batteryPcnt,0,100);

    sendBatteryLevel(batteryPcnt);                                                        //
    send(msgAI1.set(avg_mesure, 2)); 
    send_temp();                                                                          //
    send_hum();                                                                           //
    send_baro();                                                                          //
    sleep(SLEEP_TIME);                                                                    // go to sleep
}