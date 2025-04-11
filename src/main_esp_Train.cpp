//!---------------------       Inclusões de bibliotecas ---------------------

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "PubSubClient.h"
#include "env.h"

#include "topics.h"         //* MQTT topics
#include "pinout_config.h"  //* Pinout

//!---------------------       Definições de variáveis globais   ---------------------

int currentSpeed = 0;
int receivedSpeed = 0;

//!---------------------       Cabeçalho de Funções     ---------------------

void callback(char* topic, byte* message, unsigned int length);
void connectToMQTT();
void connectToWiFi();
void statusLED(byte status);
void turnOffLEDs();
void handleError();
void setSpeed(int speed);

//!---------------------       Definições de Constantes ---------------------

WiFiClientSecure client;
PubSubClient mqttClient(client);

//!---------------------       Loops Principais        ---------------------

void setup() {
    Serial.begin(115200);
    // As the free tier of HiveMQ does not allow generating a CA, it is necessary
    // to disable certificate verification
    client.setInsecure();

    // H-Bridge
    ledcSetup(PWM_CHANNEL_FORWARD, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_BACKWARD, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(FORWARD_DIRECTION_PIN, PWM_CHANNEL_FORWARD);
    ledcAttachPin(BACKWARD_DIRECTION_PIN, PWM_CHANNEL_BACKWARD);
    ledcWrite(PWM_CHANNEL_FORWARD, 0);
    ledcWrite(PWM_CHANNEL_BACKWARD, 0);
    digitalWrite(FORWARD_DIRECTION_PIN, LOW);
    digitalWrite(BACKWARD_DIRECTION_PIN, LOW);

    // Status LED

    ledcSetup(PWM_CHANNEL_LED_R, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_LED_G, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_LED_B, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(STATUS_LED_R_PIN, PWM_CHANNEL_LED_R);
    ledcAttachPin(STATUS_LED_G_PIN, PWM_CHANNEL_LED_G);
    ledcAttachPin(STATUS_LED_B_PIN, PWM_CHANNEL_LED_B);
    turnOffLEDs();
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
    Serial.println(mqttClient.state());
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
        WiFi.setSleep(true);
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

        String NODE_ID = "NODE_TRAIN-";
        NODE_ID += String(random(0xffff), HEX);
        if (mqttClient.connect(NODE_ID.c_str(), MQTT_USER_CONN, MQTT_PASSWORD_CONN)) {
            mqttClient.subscribe(topicTrainSpeed);
            mqttClient.setCallback(callback);

            Serial.println("Conectado ao Broker MQTT");
            Serial.print("Inscrito no tópico: ");
            Serial.println(topicTrainSpeed);
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
    mqttClient.publish(topicTrainStatus, "Valor invalido");
}

void callback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (int i = 0; i < length; i++) {
        char c = (char)payload[i];
        message += c;
    }
    
    try
    {
        receivedSpeed = message.toInt();
        Serial.println("velocidade recebida: " + String(receivedSpeed));
    }
    catch(const std::exception& e)
    {
        handleError();
        return;
    }
    
    
    
    if (receivedSpeed >= 0 && receivedSpeed <= 200) {
        if (receivedSpeed != currentSpeed) {
            currentSpeed = receivedSpeed;
            setSpeed(receivedSpeed);
            Serial.println(String("Velocidade alterada para: ") + receivedSpeed);
        }
    }
    else {
        handleError();
        statusLED(3);
    }
}

void setSpeed(int speed) {
    ledcWrite(PWM_CHANNEL_FORWARD, 0);
    ledcWrite(PWM_CHANNEL_BACKWARD, 0);
    if (speed > 0) {
        statusLED(3);
        for (int i = 0; i <= speed; i = i+3) {
            ledcWrite(PWM_CHANNEL_FORWARD, i);
            delay(5);
        }
    }
    else if (speed < 0) {
        statusLED(4);
        for (int i = 0; i >= speed; i--) {
            ledcWrite(PWM_CHANNEL_BACKWARD, -i);
            delay(5);
        }
    }
    else {
        turnOffLEDs();
        digitalWrite(PWM_CHANNEL_FORWARD,0);
        digitalWrite(PWM_CHANNEL_BACKWARD, 0);
        ledcWrite(PWM_CHANNEL_FORWARD, 0);
        ledcWrite(PWM_CHANNEL_BACKWARD, 0);
    }
}