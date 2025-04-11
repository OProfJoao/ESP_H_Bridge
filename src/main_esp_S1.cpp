//!---------------------       Inclusões de bibliotecas ---------------------

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "DHT.h"
#include "Adafruit_Sensor.h"
#include "Ultrasonic.h"
#include "PubSubClient.h"
#include "env.h"

#include "topics.h"         //* MQTT topics
#include "pinout_config.h"  //* Pinout

//?---------------------    Periféricos neste ESP   ---------------------

//?     LDR
//?     DHT
//?     ULTRASSONICO

//?     LED STATUS
//?     LED ILUMINAÇÃO


//!---------------------       Definições de variáveis globais   ---------------------

//ultrasonic
bool detected = false;
unsigned long lastPresenceDetection = 0;

//LDR
unsigned long lastLightReading = 0;
bool lastLightStatus = false;

//dht
unsigned long lastTempReading = 0;


//!---------------------       Cabeçalho de Funções     ---------------------

void callback(char* topic, byte* message, unsigned int length);
void connectToMQTT();
void connectToWiFi();
void statusLED(byte status);
void turnOffLEDs();
void handleError();
void nodeIlumination(bool status);


//!---------------------       Definições de Constantes ---------------------

WiFiClientSecure client;
PubSubClient mqttClient(client);

Ultrasonic ultrasonic(ULTRA_1_TRIGG, ULTRA_1_ECHO);
DHT dht(DHT_PIN, DHT11);

//!---------------------       Loops Principais        ---------------------

void setup() {
    Serial.begin(115200);
    // As the free tier of HiveMQ does not allow generating a CA, it is necessary
    // to disable certificate verification
    client.setInsecure();

    //DHT11
    dht.begin();

    // Status LED
    ledcSetup(PWM_CHANNEL_LED_R, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_LED_G, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_LED_B, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(STATUS_LED_R_PIN, PWM_CHANNEL_LED_R);
    ledcAttachPin(STATUS_LED_G_PIN, PWM_CHANNEL_LED_G);
    ledcAttachPin(STATUS_LED_B_PIN, PWM_CHANNEL_LED_B);
    turnOffLEDs();

    pinMode(LEDPIN, OUTPUT);

    nodeIlumination(0);
    delay(2000);
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        handleError();
        connectToWiFi();
    }
    if (!mqttClient.connected()) {
        handleError();
        connectToMQTT();
    }
    mqttClient.loop();

    unsigned long currentTime = millis();

    //TODO: Read luminance sensor data and publish it to the luminance topic
    Serial.print("Reading Luminance sensor: ");
    byte luminanceValue = map(analogRead(LDR_PIN), 0, 4095, 0, 100);
    Serial.println(luminanceValue);

    if (luminanceValue < 20 && !lastLightStatus && (currentTime - lastLightReading > 5000)) {
        lastLightStatus = true;
        lastLightReading = currentTime;
        mqttClient.publish(topicLuminanceSensor, String("1").c_str());
    }
    if (luminanceValue >= 20 && lastLightStatus && (currentTime - lastLightReading > 5000)) {
        lastLightStatus = false;
        lastLightReading = currentTime;
        mqttClient.publish(topicLuminanceSensor, String("0").c_str());
    }


    //TODO: Read temperatura/humidity sensor data and publish it to the temperatura/humidity topic 
    if ((currentTime - lastTempReading > 3000)) {
        Serial.print("Reading DHT sensor: ");
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();
        Serial.print(temperature); Serial.print(" | ");
        Serial.println(humidity);
        if ((!isnan(temperature) || !isnan(humidity))) {
            lastTempReading = currentTime;
            mqttClient.publish(topicTemperatureSensor, String(temperature).c_str());
            mqttClient.publish(topicHumiditySensor, String(humidity).c_str());
        }
    }


    //TODO: Read distance sensor data and publish it to the presence topic
    Serial.print("Reading Distance sensor: ");
    long microsec = ultrasonic.timing();
    float distance = ultrasonic.convert(microsec, Ultrasonic::CM);
    Serial.println(distance);

    if (distance < 10 && detected == false && (currentTime - lastPresenceDetection >= 3000)) {
        mqttClient.publish(topicPresenceSensor1, String("1").c_str());
        detected = true;
        lastPresenceDetection = currentTime;
    }
    if (distance > 10 && detected == true && (currentTime - lastPresenceDetection >= 3000)) {
        detected = false;
        lastPresenceDetection = currentTime;
        mqttClient.publish(topicPresenceSensor1, String("0").c_str());
    }
    delay(500);
    Serial.println("\n\n");
}

//!---------------------       Funções extras        ---------------------

void setLEDColor(byte r, byte g, byte b) {
    ledcWrite(PWM_CHANNEL_LED_R, r);
    ledcWrite(PWM_CHANNEL_LED_G, g);
    ledcWrite(PWM_CHANNEL_LED_B, b);
}

void connectToWiFi() {
    statusLED(1);
    delay(500);
    WiFi.begin(WIFI_CONN_SSID, WIFI_CONN_PASSWORD);
    Serial.print("Conectando ao WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Conectando ao WiFi: ");
        Serial.println(String(WIFI_CONN_SSID) + " / " + String(WIFI_CONN_PASSWORD));
    }
    else {
        Serial.println("Falha ao conectar ao WiFi!");
    }
}

void connectToMQTT() {
    statusLED(2);
    delay(500);
    mqttClient.setServer(MQTT_BROKER_CONN, MQTT_PORT_CONN);

    while (!mqttClient.connected()) {
        Serial.print("Conectando ao Broker MQTT...");
        String NODE_ID = "NODE_1-";
        NODE_ID += String(random(0xffff), HEX);
        if (mqttClient.connect(NODE_ID.c_str(), MQTT_USER_CONN, MQTT_PASSWORD_CONN)) {
            Serial.println("Conectado ao Broker MQTT");


            mqttClient.subscribe(topicLuminanceSensor);
            mqttClient.setCallback(callback);
            Serial.print("Inscrito no tópico: ");
            Serial.print(topicLuminanceSensor);
            turnOffLEDs();
        }
        else {
            Serial.println("Falha ao conectar ao Broker MQTT");
            Serial.print("Erro: ");
            Serial.println(mqttClient.state());
            delay(2000);
            Serial.println("Tentando novamente...");
        }
    }
}

void statusLED(byte status) {
    turnOffLEDs();
    switch (status) {
    case 254:  // Erro (Vermelho)
        setLEDColor(255, 0, 0);
        break;

    case 1:  // Conectando ao Wi-Fi (Amarelo)
        setLEDColor(150, 255, 0);
        break;

    case 2:  // Conectando ao MQTT (Rosa)
        setLEDColor(150, 0, 255);
        break;

    case 3:  // Movendo para frente (Verde)
        setLEDColor(0, 255, 0);
        break;

    case 4:  // Movendo para trás (Ciano)
        setLEDColor(0, 255, 255);
        break;

    default:
        for (byte i = 0; i < 4; i++) {
            setLEDColor(0, 0, 255);  // erro no status (pisca azul)
            delay(100);
            turnOffLEDs();
            delay(100);
        }
        break;
    }
}

void turnOffLEDs() { setLEDColor(0, 0, 0); }

void handleError() {
    for (byte i = 0; i < 4; i++) {
        statusLED(254);
        delay(100);
        turnOffLEDs();
        delay(100);
    }
    turnOffLEDs();
}

void callback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    bool error = false;
    for (int i = 0; i < length; i++) {
        char c = (char)payload[i];
        if (!isDigit(c)) {
            handleError();
            error = true;
            return;
        }
        message += c;
    }

    if (!error) {
        Serial.print("Received on: ");
        Serial.print(String(topic) + "=> ");
        Serial.println(String(message));

        if (String(message) == "1") {
            nodeIlumination(true); //Acende os leds
        }
        else if (String(message) == "0") {
            nodeIlumination(false); //Apaga os leds
        }
        else {
            handleError();
            statusLED(3);
        }
    }
}

void nodeIlumination(bool status) {
    Serial.println(status);
    digitalWrite(LEDPIN, status);
}