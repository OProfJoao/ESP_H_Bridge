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

#define ULTRA_ECHO 26
#define ULTRA_TRIGG 27


#define LEDPIN 25

#define SERVO_1_PIN 32
#define SERVO_2_PIN 33


//!---------------------       Definições de variáveis     ---------------------

//ultrasonic
bool ultra_detected = false;
unsigned long ultra_lastDetection = 0;



//!---------------------       Cabeçalho de Funções     ---------------------

void callback(char* topic, byte* message, unsigned int length);
void connectToMQTT();
void connectToWiFi();
void statusLED(byte status);
void turnOffLEDs();
void handleError();
void servoPosition(bool position, Servo& servo);
void nodeIlumination(bool status);
void readUltrasonic1();

//!---------------------       Definições de Constantes ---------------------

WiFiClientSecure client;
PubSubClient mqttClient(client);

Ultrasonic ultrasonic(ULTRA_ECHO, ULTRA_TRIGG);


Servo servo1;
Servo servo2;

//TODO: Configurar valores corretos
#define POSITION_0_ANGLE 120
#define POSITION_1_ANGLE 60

//!---------------------       Definição dos tópicos        ---------------------

//Publish
// const char* topicPresenceSensor = "ferrorama/station/presence3";


// const char* topicLuminanceStatus = "ferrorama/station/luminanceStatus";
// const char* topicServo1Position = "ferrorama/servo1/position";
// const char* topicServo2Position = "ferrorama/servo2/position";


//!---------------------       Loops Principais        ---------------------

void setup() {
    Serial.begin(115200);
    // As the free tier of HiveMQ does not allow generating a CA, it is necessary
    // to disable certificate verification
    client.setInsecure();

    servo1.attach(SERVO_1_PIN);
    servo2.attach(SERVO_2_PIN);

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



    //TODO: Read distance sensor data and publish it to the presence topic
    unsigned long currentTime = millis();

    Serial.print("Reading Distance sensor: ");
    long microsec = ultrasonic.timing();
    float distance = ultrasonic.convert(microsec, Ultrasonic::CM);
    Serial.println(distance);

    if (distance < 10 && ultra_detected == false && (currentTime - ultra_lastDetection >= 3000)) {
        mqttClient.publish(topicPresenceSensor3, String("1").c_str());
        ultra_detected = true;
        ultra_lastDetection = currentTime;
    }
    if (distance > 10 && ultra_detected == true && (currentTime - ultra_lastDetection >= 3000)) {
        mqttClient.publish(topicPresenceSensor3, String("0").c_str());
        ultra_detected = false;
        ultra_lastDetection = currentTime;
    }

    // Serial.print("Reading Distance sensor: ");
    // long microsec = ultrasonic.timing();
    // float distance = ultrasonic.convert(microsec, Ultrasonic::CM);
    // Serial.println(distance);

    // if (distance < 10 && detected == false && (currentTime - lastPresenceDetection >= 3000)) {
    //     mqttClient.publish(topicPresenceSensor, String("1").c_str());
    //     detected = true;
    //     lastPresenceDetection = currentTime;
    // }
    // if (distance > 10 && detected == true && (currentTime - lastPresenceDetection >= 3000)) {
    //     detected = false;
    //     lastPresenceDetection = currentTime;
    //     mqttClient.publish(topicPresenceSensor, String("0").c_str());
    // }
    // readUltrasonic1();
    delay(500);
}

//!---------------------       Funções extras        ---------------------

void readUltrasonic1() {

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
        Serial.println("Conectando ao Broker MQTT...");
        String NODE_ID = "NODE_1-";
        NODE_ID += String(random(0xffff), HEX);
        if (mqttClient.connect(NODE_ID.c_str(), MQTT_USER_CONN, MQTT_PASSWORD_CONN)) {
            Serial.println("Conectado ao Broker MQTT : " + String(NODE_ID));


            mqttClient.subscribe(topicLuminanceSensor);
            mqttClient.setCallback(callback);
            Serial.print("Inscrito no tópico: ");
            Serial.println(topicLuminanceSensor);
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
                nodeIlumination(true); //Acende os leds
            }
            else if (message == "0") {
                nodeIlumination(false); //Apaga os leds
            }
            else {
                handleError();
                statusLED(3);
            }
        }
        if (String(topic) == topicServoPosition2) {
            Serial.println("LightStatus: " + String(message));
            if (message == "1") {
                servoPosition(1, servo1); //Acende os leds
            }
            else if (message == "0") {
                servoPosition(0, servo1); //Apaga os leds
            }
            else {
                handleError();
                statusLED(3);
            }
        }
        if (String(topic) == topicServoPosition3) {
            if (message == "1") {
                servoPosition(1, servo2); //Acende os leds
            }
            else if (message == "0") {
                servoPosition(0, servo2); //Apaga os leds
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

void servoPosition(bool position, Servo& servo) {
    if (position == 0) {
        statusLED(3);
        servo.write(POSITION_0_ANGLE);
    }
    else {
        statusLED(4);
        servo.write(POSITION_1_ANGLE);
    }
}