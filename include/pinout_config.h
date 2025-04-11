#ifndef PINOUT
#define PINOUT

//! ---- Default PWM setting ----

#define PWM_FREQ 500
#define PWM_RESOLUTION 8

#define PWM_CHANNEL_FORWARD 0
#define PWM_CHANNEL_BACKWARD 1

#define PWM_CHANNEL_LED_R 2
#define PWM_CHANNEL_LED_G 3
#define PWM_CHANNEL_LED_B 4

//! ---- Default Servo angle setting ----

//TODO: Configurar valores corretos
#define POSITION_0_ANGLE 120
#define POSITION_1_ANGLE 60



//*---------------------    Physical Pins   ---------------------

//* ---- RGB LED ----
#define STATUS_LED_R_PIN 21
#define STATUS_LED_G_PIN 22
#define STATUS_LED_B_PIN 23

//* ---- Ultrasonic Sensors ----
#define ULTRA_1_ECHO 26
#define ULTRA_1_TRIGG 27


#define ULTRA_2_ECHO 18
#define ULTRA_2_TRIGG 5

//* ---- Sensors ----
#define LDR_PIN 34      
#define DHT_PIN 32

//* ---- Illumination LED ----
#define LEDPIN 13

//* ---- Servo Motors ----
#define SERVO_1_PIN 32
#define SERVO_2_PIN 33

//* ---- H-Bridge Motor Driver ----
#define FORWARD_DIRECTION_PIN 32   
#define BACKWARD_DIRECTION_PIN 33  

#endif
