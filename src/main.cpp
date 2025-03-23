#include <Arduino.h>
#include <WiFi.h>
#include "PubSubClient.h"
#include "env.h"
#include <WiFiClientSecure.h>

#define GPIOPIN32 32
#define GPIOPIN33 33

#define STATUS_LED 2

#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1
#define PWM_FREQ 500
#define PWM_RESOLUTION 8

WiFiClientSecure client;
PubSubClient mqttClient(client);

//Values set in a env file

const char *ssid = wifi_ssid;
const char *pass = wifi_password;

const char *broker = mqtt_broker;

const char *mqtt_user = mqtt_username;
const char *mqtt_pass = mqtt_password;

//----------------------------------------------

const int port = 8883;

const char *topic = "esp_motor/speed";

void connectToWIFI();

void callback(char *topic, byte *message, unsigned int length);
void connectToBroker();
void statusBlink();

void setup()
{
    ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(GPIOPIN32, PWM_CHANNEL_A);
    ledcAttachPin(GPIOPIN33, PWM_CHANNEL_B);
    ledcWrite(PWM_CHANNEL_A, 0);
    ledcWrite(PWM_CHANNEL_B, 0);

    pinMode(STATUS_LED, OUTPUT); // LED de status
    digitalWrite(STATUS_LED, LOW);

    Serial.begin(115200);

    //Como o hivemq grátis não permite gerar um CA, é necessário desabilitar a verificação de certificado
    client.setInsecure();


    mqttClient.setServer(broker, port);
    mqttClient.setCallback(callback);

    digitalWrite(GPIOPIN32, LOW);
    digitalWrite(GPIOPIN33, LOW);
    connectToWIFI();
    connectToBroker();
}

void loop()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        digitalWrite(STATUS_LED,LOW);
        connectToWIFI();
    }
    else
    {
        digitalWrite(STATUS_LED, HIGH);
    }
    if (!mqttClient.connected())
    {
        digitalWrite(STATUS_LED,LOW);
        connectToBroker();
    }
    else
    {
        digitalWrite(STATUS_LED, HIGH);
    }
    mqttClient.loop();
}

/*-------------------------------------------------------------------------------*/

void connectToWIFI()
{
    WiFi.begin(ssid, pass);
    Serial.println("Connecting to Wifi...");

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.print("Status: ");
        Serial.println(WiFi.status());
    }
    Serial.println("Wifi Connected");
    statusBlink();
}
void statusBlink()
{
    for (int i = 0; i < 6; i++)
    {
        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
        delay(500);
    }
}

void connectToBroker()
{
    Serial.println("Connecting to the Broker...");
    while (!mqttClient.connected())
    {
        String clientId = "ESP32-Servo-" + String(random(0xffff), HEX);
        Serial.print("Attempting connection as ");
        Serial.println(clientId);
        if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass))
        {
            Serial.println("Connected to the Broker!");
            mqttClient.subscribe(topic);
            Serial.print("Subscribed to topic: ");
            Serial.println(topic);
            statusBlink();
        }
        else
        {
            Serial.print("Connection failed, code: ");
            Serial.println(mqttClient.state());
            delay(5000); // Adicionado delay para evitar loop rápido
        }
    }
}

void callback(char *topic, byte *payload, unsigned int length)
{
    String message = "";

    for (int i = 0; i < length; i++)
    {
        char c = (char)payload[i];
        if (!isDigit(c))
        {
            mqttClient.publish("esp_motor/status", "Valor invalido");
            return;
        }
        message += c;
    }

    int speed = message.toInt();
    if (speed > 0 && speed < 255)
    {
        ledcWrite(PWM_CHANNEL_A, speed);
    }
    else
    {
        mqttClient.publish("esp_motor/status", "Valor invalido");
    }
}