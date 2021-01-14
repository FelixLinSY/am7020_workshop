#include "config.h"
#include <PubSubClient.h>
#include <TinyGsmClient.h>
#include "DHT.h"   //REQUIRES DHT Sensor & Adafruit Unified Sensor Library

#define DHTPIN  7
#define DHTTYPE DHT11   // one of DHT11/DHT21/DHT22
DHT             dht(DHTPIN, DHTTYPE);

#ifdef DEBUG_DUMP_AT_COMMAND 
#include <StreamDebugger.h>
StreamDebugger  debugger(SerialAT, SerialMon);
TinyGsm         modem(debugger, AM7020_RESET);

#else
TinyGsm         modem(SerialAT, AM7020_RESET);

#endif

TinyGsmClient tcpClient(modem);
PubSubClient  mqttClient(MQTT_BROKER, MQTT_PORT, tcpClient);

void mqttCallback(char *topic, byte *payload, unsigned int len);
void mqttConnect(void);
void nbConnect(void);

void setup()
{
    SerialMon.begin(MONITOR_BAUDRATE);
    SerialAT.begin(AM7020_BAUDRATE);
    
    randomSeed(analogRead(A0));
    
    pinMode(LED_BUILTIN, OUTPUT);
  
    nbConnect();
    mqttClient.setCallback(mqttCallback);
    
    dht.begin();
}

void getDHT(float *humi, float *temp) {
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    float f = dht.readTemperature(true);
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) || isnan(f)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }

    *humi = h;
    *temp = t;

    // Compute heat index in Fahrenheit (the default)
    float hif = dht.computeHeatIndex(f, h);
    // Compute heat index in Celsius (isFahreheit = false)
    float hic = dht.computeHeatIndex(t, h, false);

    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.print(F("°C "));
    Serial.print(f);
    Serial.print(F("°F  Heat index: "));
    Serial.print(hic);
    Serial.print(F("°C "));
    Serial.print(hif);
    Serial.println(F("°F"));
}
void loop()
{
    static unsigned long    nb_timer = 0, dht_timer = 0;
    char                    buff[16];
    static float            humidity, temperature;

    if (!mqttClient.connected()) {
        if (!modem.isNetworkConnected()) {
            nbConnect();
        }
        SerialMon.println(F("=== MQTT NOT CONNECTED ==="));
        mqttConnect();
    }
    
    if (millis() >= dht_timer) {
        dht_timer = millis() + 10000;   // get data every 10 sec
        getDHT(&humidity, &temperature);
    }
    
    if (millis() >= nb_timer) {
        nb_timer = millis() + UPLOAD_INTERVAL;
        // temperature
        sprintf(buff, "%s", String(temperature).c_str());
        SerialMon.print(F("[Publish]-Topic: "));
        SerialMon.print(MQTT_TOPIC_T);
        SerialMon.print(F(", data: "));
        SerialMon.println(buff);
        mqttClient.publish(MQTT_TOPIC_T, buff);

        // humidity
        sprintf(buff, "%s", String(humidity).c_str());
        SerialMon.print(F("[Publish]-Topic: "));
        SerialMon.print(MQTT_TOPIC_H);
        SerialMon.print(F(", data: "));
        SerialMon.println(buff);
        mqttClient.publish(MQTT_TOPIC_H, buff);
    }
    
    mqttClient.loop();
}

void mqttCallback(char *topic, byte *payload, unsigned int len)
{
    char buff[256];
    SerialMon.print(F("[Subscribe]-Topic: "));
    SerialMon.print(topic);
    SerialMon.print(F(", data]: "));
    SerialMon.write(payload, len);
    SerialMon.println();
    for (int i=0; i<len; i++) {
        buff[i] = payload[i];
    }
    if (strncmp(buff, "ON", len) == 0) {
        digitalWrite(LED_BUILTIN, HIGH);
    } else if (strncmp(buff, "OFF", len) == 0) {
        digitalWrite(LED_BUILTIN, LOW);
    } 
}

void mqttConnect(void)
{
    SerialMon.print(F("Connecting to "));
    SerialMon.print(MQTT_BROKER);
    SerialMon.print(F("..."));

    // Connect to MQTT Broker
    String mqttid = ("MQTTID_" + String(random(65536)));
    while (!mqttClient.connect(mqttid.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
        SerialMon.println(F(" fail"));
    }
    SerialMon.println(F(" success"));
    mqttClient.subscribe(MQTT_TOPIC_LED);
}

void nbConnect(void)
{
    SerialMon.println(F("Initializing modem..."));
    while (!modem.init() || !modem.nbiotConnect(APN, BAND)) {
        SerialMon.print(F("."));
    };

    SerialMon.print(F("Waiting for network..."));
    while (!modem.waitForNetwork()) {
        SerialMon.print(F("."));
    }
    SerialMon.println(F(" success"));
}
