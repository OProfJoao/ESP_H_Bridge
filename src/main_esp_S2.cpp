//!---------------------       Inclusões de bibliotecas ---------------------

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "Ultrasonic.h"
#include "ESP32SERVO.h"

#include "PubSubClient.h"
#include "env.h"
#include "topics.h"

//!---------------------       Definição dos pinos      ---------------------

#define PWM_FREQ 500
#define PWM_RESOLUTION 8

#define STATUS_LED_R_PIN 25
#define STATUS_LED_G_PIN 26
#define STATUS_LED_B_PIN 27

#define PWM_CHANNEL_LED_R 2
#define PWM_CHANNEL_LED_G 3
#define PWM_CHANNEL_LED_B 4



//TODO: Configurar pinos corretos

#define ULTRA_1_ECHO 0
#define ULTRA_1_TRIGG 1

#define ULTRA_2_ECHO 0
#define ULTRA_2_TRIGG 1

#define LEDPIN 30

#define SERVO_PIN 10

//!---------------------       Definições de variáveis     ---------------------

//ultrasonic
bool ultra_1_detected = false;
unsigned long ultra_1_lastDetection = 0;

bool ultra_2_detected = false;
unsigned long ultra_2_lastDetection = 0;

//!---------------------       Cabeçalho de Funções     ---------------------

void callback(char* topic, byte* message, unsigned int length);
void connectToMQTT();
void connectToWiFi();
void statusLED(byte status);
void turnOffLEDs();
void handleError();
void servoPosition(bool position);
void nodeIlumination(bool status);

//!---------------------       Definições de Constantes ---------------------

WiFiClientSecure client;
PubSubClient mqttClient(client);

Ultrasonic ultrasonic1(ULTRA_1_ECHO, ULTRA_1_TRIGG);
Ultrasonic ultrasonic2(ULTRA_2_ECHO, ULTRA_2_TRIGG);

Servo servo;

//TODO: Configurar valores corretos
#define POSITION_0_ANGLE 120
#define POSITION_1_ANGLE 60

//!---------------------       Definição dos tópicos        ---------------------

//Publish
// const char* topicPresenceSensor2 = "ferrorama/station/presence2";
// const char* topicPresenceSensor4 = "ferrorama/station/presence4";

// const char* topicLuminanceStatus = "ferrorama/station/luminanceStatus";
// const char* topicServoPosition = "ferrorama/servo/position";


//!---------------------       Loops Principais        ---------------------

void setup() {
    Serial.begin(115200);
    // As the free tier of HiveMQ does not allow generating a CA, it is necessary
    // to disable certificate verification
    client.setInsecure();

    servo.attach(SERVO_PIN);

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

    //TODO: Read distance sensor data and publish it to the presence topic
    readUltrasonic1(currentTime);
    readUltrasonic2(currentTime);
}

//!---------------------       Funções extras        ---------------------

void readUltrasonic1(unsigned long currentTime) {

    long microsec1 = ultrasonic1.timing();
    float distance = ultrasonic1.convert(microsec1, Ultrasonic::CM);
    if (distance < 10 && ultra_1_detected == false && (currentTime - ultra_1_lastDetection >= 3000)) {
        mqttClient.publish(topicPresenceSensor2, String("1").c_str());
        ultra_1_detected = true;
        ultra_1_lastDetection = currentTime;
    }
    if (distance > 10 && ultra_1_detected == true && (currentTime - ultra_1_lastDetection >= 3000)) {
        ultra_1_detected = false;
        ultra_1_lastDetection = currentTime;
    }
}

void readUltrasonic2(unsigned long currentTime){
    
    long microsec2 = ultrasonic2.timing();
    float distance = ultrasonic2.convert(microsec2, Ultrasonic::CM);
    if (distance < 10 && ultra_2_detected == false && (currentTime - ultra_2_lastDetection >= 3000)) {
        mqttClient.publish(topicPresenceSensor4, String("1").c_str());
        ultra_2_detected = true;
        ultra_2_lastDetection = currentTime;
    }
    if (distance > 10 && ultra_2_detected == true && (currentTime - ultra_2_lastDetection >= 3000)) {
        ultra_2_detected = false;
        ultra_2_lastDetection = currentTime;
    }
}


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
        if (String(topic) == topicLuminanceSensor) {
            if (message == "1") {
                nodeIlumination(1); //Acende os leds
            }
            else if (message == "0") {
                nodeIlumination(0); //Apaga os leds
            }
            else {
                handleError();
                statusLED(3);
            }
        }
        if (String(topic) == topicServoPosition1) {
            if (message == "1") {
                servoPosition(1); 
            }
            else if (message == "0") {
                servoPosition(0); 
            }
            else {
                handleError();
                statusLED(3);
            }
        }
    }
}

void nodeIlumination(bool status) {
    digitalWrite(LEDPIN, status);
}

void servoPosition(bool position){
    if(position == 0){
        statusLED(3);
        servo.write(POSITION_0_ANGLE);
    }
    else{
        statusLED(4);
        servo.write(POSITION_1_ANGLE);
    }
}