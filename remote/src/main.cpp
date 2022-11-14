#include <SPI.h>
#include <EEPROM.h>
#include <math.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

//------------ Turn on generic Debug Prints
#define DEBUG_PRINTS

//------------ Turn on various kinds of debug prints
// #define DEBUG_FAILS
#define DEBUG_SENT
// #define DEBUG_READS
// #define DEBUG_EXPO
// #define DEBUG_NOISE

// Data sent to the receiver
typedef struct
{
    int16_t speedmotorLeft;
    int16_t speedmotorRight;
    int8_t Fire;
} packet_t;

packet_t sentData;
packet_t recData;

//------ ESP_NOW Variables ------
// Robot MAC address
uint8_t robotAddress[] = {0xF4, 0x12, 0xFA, 0x50, 0x6D, 0xC8};
String success;

esp_now_peer_info_t peerInfo;
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    // Serial.print("\r\nLast Packet Send Status:\t");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    if (status == 0)
    {
        success = "Delivery Success :)";
    }
    else
    {
        success = "Delivery Fail :(";
    }
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    memcpy(&recData, incomingData, sizeof(recData));
}

// standard remote
const int steerPot = 9;
const int accPot = 10;

const int fireBtn = 36;
const int ledPin = 13;

const int potStrRightEnd = 3;   // default 0, reversed 1024
const int potStrLeftEnd = 1020; // default 1024, reversed 0

const int potAccForwardEnd = 1020; // default 0, reversed 1024
const int potAccBackEnd = 3;       // default 1024, reversed 0

const int potStrRightStart = 450; // default 512
const int potStrLeftStart = 505;  // default 512

const int potAccForwardStart = 485; // default 512
const int potAccBackStart = 435;    // default 512

// customisable vars
int PWMmax = 255;
int analogReadMax = 1023;
int analogRes = 10;

int strExpoalpha = 100;
int accExpoalpha = 100;
int wpnAccel = 8;

// variables for the sketch
int right = 0;
int left = 0;
int forward = 0;
int back = 0;
int wpn = 0;
int strPWMmax = PWMmax;
int accPWMmax = PWMmax;
int trimValue = 0;
int address = 0;
bool memSetted = true;
int rev_str = false;
int debug_min = 1023;
int debug_max = 0;
float leverValue_f = 0.0;
int leverValue = 0;

bool wpnSafetyCeck = true;
bool firing = false;
unsigned long current_time = 0;
unsigned long animation_millis_1 = 0;
unsigned long animation_millis_2 = 0;
bool topHold = false;
bool leverMode = false;
int temp_value_debug = 0;
int temp_curve_debug = 0;

void setup()
{
    analogReadResolution(analogRes);
    analogSetAttenuation(ADC_11db);
    pinMode(fireBtn, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);
#ifdef DEBUG_PRINTS
    Serial.begin(115200);
#endif

    Serial.println("RAC GENERIC BOT");

    //---------------------------------------ESP NOW setup
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_send_cb(OnDataSent);
    memcpy(peerInfo.peer_addr, robotAddress, 6);
    esp_wifi_set_channel(10, WIFI_SECOND_CHAN_NONE);
    peerInfo.channel = 10;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
    // read pots values
    int strValue = analogRead(steerPot);
    delay(3);
    int accValue = analogRead(accPot);
    delay(3);
    current_time = millis();

    // Check Fire button pressed
    bool fireValue = !digitalRead(fireBtn);

    //----------------------------------------------------WEAPON CODE
    sentData.Fire = false;

    if (fireValue)
    {
        sentData.Fire = true;
    }

    //----------------------------------------------------MOTOR CODE
    // setting dual rates

    // map the value to useful pwm-friendly ones
    right = map(strValue, potStrRightStart, potStrRightEnd, 0, strPWMmax);
    left = map(strValue, potStrLeftStart, potStrLeftEnd, 0, strPWMmax);
    forward = map(accValue, potAccForwardStart, potAccForwardEnd, 0, accPWMmax);
    back = map(accValue, potAccBackStart, potAccBackEnd, 0, -accPWMmax);

    // correct the values in case of wrong pot limits on initialization and
    // so all of them will have the same weight in next computations
    right = constrain(right, 0, strPWMmax);
    left = constrain(left, 0, strPWMmax);
    forward = constrain(forward, 0, accPWMmax);
    back = constrain(back, -accPWMmax, 0);

    // recorrect the data not to have more than max +PWM while non pivot-steering
    int left_pwm = constrain(forward + back + right - left, -PWMmax, PWMmax);
    int right_pwm = constrain(forward + back - right + left, -PWMmax, PWMmax);

    sentData.speedmotorLeft = left_pwm;
    sentData.speedmotorRight = right_pwm;

    esp_err_t result = -1;
    result = esp_now_send(robotAddress, (uint8_t *)&sentData, sizeof(sentData));
    if (result == ESP_OK)
    {
        // Serial.println("Sent with success");
    }
    else
    {
        // Serial.println("Error sending the data>");
    }

    //------------------- all debug prints

    // DEBUG NOISE POT
#ifdef DEBUG_NOISE
    int check_debug = leverValue;
    if (check_debug > debug_max)
    {
        debug_max = check_debug;
    }
    if (check_debug < debug_min)
    {
        debug_min = check_debug;
    }
    Serial.print(check_debug);
    Serial.print(", ");
    Serial.print(debug_max);
    Serial.print(", ");
    Serial.print(debug_min);
    Serial.print(", ");
    Serial.println(debug_max - debug_min);
#endif

    // DEBUG FOR SENT VALUES
#ifdef DEBUG_SENT
    Serial.print("LPWM: ");
    Serial.print(sentData.speedmotorLeft);
    Serial.print("\t");
    Serial.print("R PWM: ");
    Serial.print(sentData.speedmotorRight);
    Serial.print("\t");
    int delta = 0;

    int sign_r = 1;
    if (sentData.speedmotorRight != 0)
        sign_r = sentData.speedmotorRight / sentData.speedmotorRight;
    int sign_l = 1;
    if (sentData.speedmotorLeft != 0)
        sign_l = sentData.speedmotorLeft / sentData.speedmotorLeft;

    int high_v = sentData.speedmotorRight * sign_r >= sentData.speedmotorLeft * sign_l ? sentData.speedmotorRight : sentData.speedmotorLeft;
    int low_v = sentData.speedmotorRight * sign_r <= sentData.speedmotorLeft * sign_l ? sentData.speedmotorRight : sentData.speedmotorLeft;

    if (sign_r == sign_l)
    {
        delta = high_v - low_v;
    }
    else
    {
        delta = sentData.speedmotorRight * sign_r + sentData.speedmotorLeft * sign_l;
    }

    Serial.print("delta: ");
    Serial.print(delta);
    Serial.print("\t");

    Serial.print("FIRE: ");
    Serial.println((int)sentData.Fire);
#endif

    // DEBUG FOR READ VALUES
#ifdef DEBUG_READS
    Serial.print("accpot: ");
    Serial.print(accValue);
    Serial.print("\t");

    Serial.print("strpot");
    Serial.print(strValue);
    Serial.print("\t");

    Serial.print("leverpot");
    Serial.print(leverValue);
    Serial.print("\t");

    Serial.print("BR: ");
    Serial.print(fireValue);
    Serial.print("\t");

    Serial.print("trimPot: ");
    Serial.println(trimValue);
#endif

    // DEBUG EXPO CURVE
#ifdef DEBUG_EXPO
    Serial.print("left:");
    Serial.print(left);
    Serial.print(",");
    Serial.print("expo:");
    Serial.print(strExpoalpha);
    Serial.print(",");
    temp_value_debug += 3;
    if (temp_value_debug > 255)
    {
        temp_value_debug = 0;
    }
    Serial.print("value:");
    Serial.print(temp_value_debug);
    Serial.print(",");
    int valuex = map(temp_value_debug, 0, strPWMmax, 0, 1000);
    temp_curve_debug = pow(valuex / 1000.0f, strExpoalpha / 100.0f) * strPWMmax;
    Serial.print("curve:");
    Serial.print(temp_curve_debug);
    Serial.println();
#endif
}
