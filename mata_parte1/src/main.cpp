#include <Arduino.h>
#include "ClosedCube_HDC1080.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include "ESP8266WiFi.h"
#define sensorPower 0
#define sensorPin A0

ClosedCube_HDC1080 sensor;
//SoftwareSerial gpsserial(2, 0);
WiFiClient client;


String serverIP = "54.83.141.2";
String wifi = "dlink";
String password = "";
String id = "sensor01";
String serverPort = "1026";
const uint16 iterations = 3;
float flat = 0, flon = 0;
bool newData = false;
double dataTemp = 0, promTemp = 0;
double dataHum = 0, promHum = 0;
double dataHumPlanta = 0, promHumPlanta = 0;
uint16_t countSendGPS = 0;

void sendTask();
void sendDataToServer(String patchata);
//static void smartdelay(unsigned long ms);

void setup()
{
  pinMode(sensorPower, OUTPUT);
	digitalWrite(sensorPower, LOW);

  sensor.begin(0x40);
  Serial.begin(115200);

  WiFi.begin(wifi,password);
  
  while(WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Connected");
}

void loop()
{
  sendTask();
}

void sendTask()
{
  enum class VarStatesMachine
  {
    INIT_VAR,
    TIME_VAR,
    SEND_VAR,
  };

  static VarStatesMachine varStateMachine = VarStatesMachine::INIT_VAR;
  String dataVar = "";
  String dataGPS = "";

  switch (varStateMachine)
  {
    case VarStatesMachine::INIT_VAR: {
      varStateMachine = VarStatesMachine::TIME_VAR;
      //smartdelay(100);
      break;
    }
    case VarStatesMachine::TIME_VAR: {
      const uint32_t intervalVar = 10000;
      static uint32_t previousMillisVar = 0;

      uint32_t currentMillisVar = millis();

      if (currentMillisVar - previousMillisVar >= intervalVar)
      {
        previousMillisVar = currentMillisVar;
        //countSendGPS++;
        varStateMachine = VarStatesMachine::SEND_VAR;
      }
      //smartdelay(100);
      break;
    }
    case VarStatesMachine::SEND_VAR: {
      for (uint32_t i = 0; i < iterations; i++)
      {
        dataTemp += sensor.readTemperature();
        delay(100);
      }

      for (uint32_t i = 0; i < iterations; i++)
      {
        dataHum += sensor.readHumidity();
        delay(100);
      }

      for (uint32_t i = 0; i < iterations; i++)
      {
        digitalWrite(sensorPower, HIGH);	// Turn the sensor ON
	      delay(10);							// Allow power to settle
	      dataHumPlanta += analogRead(sensorPin);	// Read the analog value form sensor
	      digitalWrite(sensorPower, LOW);	
      }

      promTemp = dataTemp / iterations;
      promHum = dataHum / iterations;
      promHumPlanta = dataHumPlanta / iterations;


      dataVar = "{\"temperatura\": {\"value\": " + String(promTemp) + ",\"type\": \"float\"}, \"humedad\": {\"value\": " + String(promHum) +  ",\"type\": \"float\"}, \"humedad_planta\": {\"value\": " + String(promHumPlanta) + ", \"type\": \"float\"}}";
      sendDataToServer(dataVar);
      Serial.println(dataVar);
      dataTemp = 0;
      promTemp = 0;
      dataHum = 0;
      promHum = 0;
	    dataHumPlanta = 0;
      promHumPlanta = 0;

      // if (countSendGPS == 3)
      // {
      //   countSendGPS = 0;
      //   dataGPS = "{\"id\":\"" + String(id) + "\"" + ",\"latitud\":\"" + String(flat) + "\"" + ",\"longitud\":\"" + String(flon) + "\"}";
      //   sendDataToServer(dataGPS);
      //   Serial.println(dataGPS);
      // }

      varStateMachine = VarStatesMachine::TIME_VAR;
      break;
    }
  default:
    break;
  }
}

void sendDataToServer(String patchData)
{
  if (client.connect(serverIP, 1026))
  {
    client.println("PATCH /v2/entities/sensor01/attrs HTTP/1.1");
    client.println("Host: " + serverIP);
    client.println("Content-Type: application/json");
    client.print("Content-Length: ");
    client.println(patchData.length());
    client.println();
    client.println(patchData);
  }
}

// static void smartdelay(unsigned long ms)
// {
//   for (unsigned long start = millis(); millis() - start < ms;)
//   {
//     while (gpsserial.available())
//     {
//       char c = gpsserial.read();
//       // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
//       if (gps.encode(c)) // Did a new valid sentence come in?
//         newData = true;
//       else
//         newData = false;
//     }
//   }
// }