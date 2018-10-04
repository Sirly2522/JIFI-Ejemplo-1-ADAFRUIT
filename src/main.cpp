#include <Arduino.h>

#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/************************* acceso WiFi *********************************/

#define WLAN_SSID       "ESP8266"
#define WLAN_PASS       "esp82661"

/************************* Informasion para Connectar a Adafruit IO *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "sirly2522"
#define AIO_KEY         "ff94fae579eb4ed18135a1f7258da2ee"

/************ configuración WiFi Client y MQTT Client ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.

WiFiClient client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** configurando un Feed para publicar ***************************************/

// El formato MQTT para AIO es de la forma: <username>/feeds/<feedname>

Adafruit_MQTT_Publish tempinstant = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/tempinstant");
Adafruit_MQTT_Publish tempprom = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/tempprom");
Adafruit_MQTT_Publish tempalert = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/tempalert");
/********************* Valores ******************************/
// Pin de puerto ADC
int outputpin= A0;

float tempinstant1=1;
float tempprom1=1;
float tempalert1=1;
int cont1=0;
int cont2=0;
/*************************** Sketch Codigo ***********************************************************/
void MQTT_connect();

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
 delay(10);

 Serial.println(F("Sensor de temperatura para monitoreo de temperatúra vía WEB"));

 // Connección a Wifi
 Serial.println(); Serial.println();
 Serial.print("Conectando a ");
 Serial.println(WLAN_SSID);

 WiFi.begin(WLAN_SSID, WLAN_PASS);
 while (WiFi.status() != WL_CONNECTED) {
   delay(500);
   Serial.print(".");
 }
 Serial.println();

 Serial.println("WiFi conectado");
 Serial.println("Dirección IP: "); Serial.println(WiFi.localIP());

}

void loop() {
    // put your main code here, to run repeatedly:

  MQTT_connect();

  int analogValue = analogRead(outputpin);
  float millivolts = (analogValue/1024.0) * 3300; //3300 es el voltage que ofrece NodeMCU
  float celsius = millivolts/10-55;
  //Serial.print("Celsius=   ");
  //Serial.println(celsius);

  tempinstant1=celsius;

  if(cont1==0)
    {
      tempprom1=tempinstant1;
      }
  if(cont1<=100)
    {
      cont1=cont1+1;
      tempprom1=((tempprom1)*cont1+tempinstant1)/(cont1+1);           //condicion para elvalor promedio antes de las primeras 100 muestras
      }
  if(cont1>101)
    {
      tempprom1=((tempprom1)*100+tempinstant1)/101;
      }
   tempalert1=-5;                                        //condicional de alerta
  if(tempinstant1>tempalert1)
  {
    Serial.print(F("\nEnviando Valor de alerta"));
    Serial.print((tempinstant1));
    Serial.print("...");
      if (! tempalert.publish(tempinstant1)) {
       Serial.println(F("Failed"));
      } else {
        Serial.println(F("OK!"));
      }
   }


  if(cont2>29)
  {
  Serial.print(F("\nEnviando Valor promedio  "));
  Serial.print((tempprom1));
  Serial.print("...");

  if (! tempprom.publish(tempprom1)) {          //publico valor promedio cada 58 segundos
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  cont2=0;
  }

  Serial.print(F("\nEnviando Valor instantaneo "));
  Serial.print((celsius));
  Serial.print("...");

  if (! tempinstant.publish(tempinstant1)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  /*
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
  */
  delay(2000);
  cont2=cont2+1;
}

void MQTT_connect() {
  int8_t ret;

  // Para si ya está conectado.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // conectado regresa valor de 0
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // espera 5 segundos
       retries--;
       if (retries == 0) {
         // Espera por el Watchdog que lo resetée
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
