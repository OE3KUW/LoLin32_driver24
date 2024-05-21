/******************************************************************
                       LoLin32 driver 24
                                                    қuran june 2024
******************************************************************/
#include <Arduino.h>
#include <stdio.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include "serviceSetIdentifier.h"
#include "soc/soc.h"                   
#include "soc/rtc_cntl_reg.h"          
#include <Arduino_JSON.h>    
#include <Adafruit_MPU6050.h>           // durch lib mit dabei  siehe platformio.ini
#include <Adafruit_Sensor.h>            // durch lib mit dabei
#include <Wire.h>
//#define FASTLED_ALL_PINS_HARDWARE_SPI
#include <FastLED.h>          


//#define ACCESSPOINT

#define TRUE                            1
#define FALSE                           0
#define LEN                             21
#define WAIT_ONE_SEC                    10000
#define WAIT_250_MSEC                   2500
#define WAIT_10_MSEC                    100
#define ON_BOARD_LED                    5
#define ON_BOARD_LED_ON                 0
#define ON_BOARD_LED_OFF                1

#define WHEEL_L                         2
#define WHEEL_R                         A4
#define WHEEL_L_DIRECTION               15 
#define WHEEL_R_DIRECTION               A5
#define BATTERY_LEVEL                   A3      // GPIO 39
#define REFV                            685.0   // factor
#define DELTA_BATTERY_LEVEL             60

#define NUM_LEDS                        4
#define DATA_PIN                        23
#define CLOCK_PIN                       18

#define TRIG_PIN                        25  
#define ECHO_PIN                        26  

#define TEST_PIN_RX2                    16


// global Variables ################################################

char ssidWord[60];
char passWord[60];

char *ssid = ssidWord;
char *password = passWord;


volatile int oneSecFlag, qSecFlag, tenMSecFlag;
volatile int vL, vR;
volatile int LDir, RDir;
int testPin;

hw_timer_t *timer = NULL;
void IRAM_ATTR myTimer(void);
volatile int startWiFi = 0;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

CRGB leds[NUM_LEDS];

const int led5 = ON_BOARD_LED; 
bool ledState = 1;

String sliderValueLeft  = "0";  
String sliderValueRight = "0";  
volatile float batteryLevel = 0.;

const uint8_t impulsL = 14;
const uint8_t impulsR = 27;


// Adafruit_MPU6050 mpu;

// prototypes ######################################################

int32_t smoothLevel(int32_t x);
String createJasonLetter(String i, String v); 

void impuls_R_isr(void);
void impuls_L_isr(void);


//  MPU6050:
//#    Wire.begin(SDA, SCL);
//#    mpu6050.begin();
//#    mpu6050.calcGyroOffsets(true);

// Magnetic field:
/*

#define SDA   21
#define SCL   22
#define addr 0x0D    // magnetic field sensor

    Wire.begin(SDA, SCL);
    Wire.beginTransmission(addr);
    Wire.write(0x0B);
    Wire.write(0x01);
    Wire.endTransmission();
    Wire.beginTransmission(addr);
    Wire.write(0x09);
    Wire.write(0x1D);
    Wire.endTransmission();

aus dem driver von 2019 ....

*/

void initSPIFFS()
{
    if (!SPIFFS.begin(true))  Serial.println("An error has occurred while mounting SPIFFS");
    else                      Serial.println("SPIFFS mounted successfully!");
}

void initWiFi()
{
    char text[LEN];

        printf("Connection to WiFi . . .");

#ifdef ACCESSPOINT
    WiFi.softAP("myLoLin32", "");  

//    IPAddress lclIP(192,168,2,219);  nicht nötig - solange nicht eine bestimmte ip gewünscht wird.
//    IPAddress gateway(192,168,2,1);
//    IPAddress subnet(255,255,255,0);
//    WiFi.softAPConfig(lclIP, gateway, subnet);  

    uint32_t ip = WiFi.softAPIP();

#else
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while ((WiFi.status() != WL_CONNECTED) && (startWiFi < 21))
    {
        delay(250);   
        printf(" .");  
        startWiFi++;
    }
    uint32_t ip = (uint32_t) WiFi.localIP();
#endif

    sprintf(text, "%u.%u.%u.%u", ip & 0xFF, (ip>>8) & 0xFF, (ip>>16) & 0xFF, (ip>>24) & 0xFF );
    printf("\nIP: %s\n", text);
}

String processor(const String& var)
{
    String ret = "";

    printf("processor: %s:\n", var);

    if (var == "STATE") 
    {
        if (digitalRead(led5) == ON_BOARD_LED_ON)
        {
            printf("-ON-");
            ledState = 1; ret = "-ON-";
        }
        else
        {
            printf("-OFF-");
            ledState = 0; ret = "-OFF-"; // wird beim ersten Druchlauf ausgegeben... 
        }
    }

    return ret;
}

void notifyClients(String state)
{
    printf("notifyClients!\n"); 
    ws.textAll(state);
}

void handleWebSocketMessage(void *arg, uint8_t * data, size_t len)
{
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
        
    String val;
    int length;


    printf("handleWebSocket: %s\n", (char*)data);

    if (info->final && info->index == 0 && info->len && info->opcode == WS_TEXT)
    {
        data[len] = 0;

        if (strcmp((char*)data, "bON") == 0)
        {
            ledState = 1;
            printf("handleWebSocketMessage: on\n");
            notifyClients(createJasonLetter("LED","1"));
        }
        else if (strcmp((char*)data, "bOFF") == 0)
        {
            ledState = 0;
            printf("handleWebSocketMessage: off\n");
            notifyClients(createJasonLetter("LED","0"));  
        }
        else if(strncmp((char*)data, "sLa", 3) == 0)
        {
            val = (const char*)data;
            length = val.length();
            sliderValueLeft = val.substring(3, length);
            vL = sliderValueLeft.toInt();
            printf("sLa: %s\n", sliderValueLeft);

            notifyClients(createJasonLetter("SLL",sliderValueLeft));
        }
        else if(strncmp((char*)data, "sLb", 3) == 0) 
        {
            val = (const char*)data;
            length = val.length();
            sliderValueRight = val.substring(3, length);
            vR = sliderValueRight.toInt();
            printf("sLa: %s\n", sliderValueRight);

            notifyClients(createJasonLetter("SLR",sliderValueRight));
        }
    }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient * client, AwsEventType type, 
             void * arg, uint8_t * data, size_t len)
{
    switch(type)
    {
        case WS_EVT_CONNECT: 
             printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        break;

        case WS_EVT_DISCONNECT:
             printf("WebSocket client #%u disconnected\n", client->id());
        break;

        case WS_EVT_DATA:
             handleWebSocketMessage(arg, data, len);
        break;

        case WS_EVT_PONG: 
        case WS_EVT_ERROR:
        break;
    }
}

void initWebSocket()
{
    ws.onEvent(onEvent);
    server.addHandler(&ws);
}

int32_t smoothLevel(int32_t x)
{
    static int32_t val[16] = {0};
    static int8_t i = 0; 
    int j; 
    int32_t sum; 
    
    val[i++ & 0xf] = x;  
    sum = 0;  
    for (j = 0; j < 16; j++) sum += val[j]; 
    sum >>= 4;

    return sum;
}

String createJasonLetter(String i, String v) 
{
    JSONVar message;
    message["id"] = i;
    message["value"] = v;
    return JSON.stringify(message);
}

//##################################################################
void setup() 
{
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
    // many thanx to Daniel Sebestyen 3AHELS 2024    
    
    Serial.begin(115200);
    printf("start!");

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, myTimer, true);
    timerAlarmWrite(timer, 100, true);  // 0.1 msec
    timerAlarmEnable(timer);

    attachInterrupt(digitalPinToInterrupt(impulsR), impuls_R_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(impulsL), impuls_L_isr, FALLING);



    oneSecFlag = qSecFlag = tenMSecFlag = FALSE; 

    pinMode(ON_BOARD_LED, OUTPUT);
    pinMode(WHEEL_L, OUTPUT);
    pinMode(WHEEL_R, OUTPUT);
    pinMode(WHEEL_L_DIRECTION, OUTPUT);
    pinMode(WHEEL_R_DIRECTION, OUTPUT);
    pinMode(BATTERY_LEVEL, INPUT);    
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT_PULLUP);  // ? PULLUP? vielleicht nicht nötig... 
//    pinMode(TEST_PIN_RX2, OUTPUT);



    digitalWrite(ON_BOARD_LED, LOW); // on ! ... blue 
    digitalWrite(WHEEL_L_DIRECTION, LOW );
    digitalWrite(WHEEL_R_DIRECTION, HIGH);
    digitalWrite(WHEEL_L, LOW); // stop !
    digitalWrite(WHEEL_R, LOW); // stop !


// Fast Leds:


    FastLED.addLeds<SK9822, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);

    testPin = digitalRead(TEST_PIN_RX2);

    if (testPin == HIGH)
    {
        leds[0] = CRGB{0, 0, 255}; // R B G
        leds[1] = CRGB{255, 255, 255};
        leds[2] = CRGB{255, 255, 255};
        leds[3] = CRGB{255, 255, 255};

        
        sprintf(ssidWord,"Wolfgang Uriel Kurans Handy");
        sprintf(passWord, "x1234567");

//        sprintf(ssidWord,"A1-A82861");
//        sprintf(passWord,  "7PMGDV96J8");

//const char *ssid = "A1-A82861";
//const char *password = "7PMGDV96J8";


    }
    else
    {
        leds[0] = CRGB{255, 0, 0}; // R B G
        leds[1] = CRGB{255, 255, 255};
        leds[2] = CRGB{255, 255, 255};
        leds[3] = CRGB{255, 255, 255};

//        sprintf(ssidWord,"A1-A82861"); at home... 
//        sprintf(passWord, "7PMGDV96J8");

//        sprintf(ssidWord,"iot122023");
//        sprintf(passWord, "iot122023secret");

        sprintf(ssidWord,"A1-A82861");
        sprintf(passWord, "7PMGDV96J8");


    }


    FastLED.show();

// Gyrosensor:

 //mpu.begin();
 /*if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }*/
  Serial.println("MPU6050 Found!");

  //setupt motion detection
  /*
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
  */




  
    LDir = 0; 
    RDir = 1;
    vL = vR = 0;

    initSPIFFS();
    initWiFi();
    initWebSocket();

    server.on("/", HTTP_GET, 
    [](AsyncWebServerRequest * request) {
        request->send(SPIFFS, "/index.html", "text/html", false, processor);
    });

    server.serveStatic("/", SPIFFS, "/");
    
    server.on("/logo", HTTP_GET, 
    [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/logo.png", "image/png");
    } );
    
    server.on("/currentValueLeft", HTTP_GET,                                 
    [](AsyncWebServerRequest *request) {
        request->send(200, "/text/plain", sliderValueLeft); 
    } );

    server.on("/currentValueRight", HTTP_GET,                                  
    [](AsyncWebServerRequest *request) {
        request->send(200, "/text/plain", sliderValueRight); //#3
    } );

    // Start server:
    server.begin();
}

void loop() 
{
    static int32_t oldLevel; 
    int32_t diffLevel, actualLevel, distance;

    

    
    ws.cleanupClients();

    if (oneSecFlag == TRUE)  // all Seconds 
    {
        oneSecFlag = FALSE;

        if (ledState == 1) digitalWrite(ON_BOARD_LED, ON_BOARD_LED_ON);  // on = LOW! 
        else               digitalWrite(ON_BOARD_LED, ON_BOARD_LED_OFF); // off


        actualLevel = smoothLevel(analogRead(BATTERY_LEVEL)); // glätten .... 

        diffLevel = (actualLevel > oldLevel) ? (actualLevel - oldLevel): (oldLevel - actualLevel);

        if (diffLevel > DELTA_BATTERY_LEVEL) 
        {
            oldLevel = actualLevel;
            batteryLevel = actualLevel / REFV;
            printf("battery level: %0.2f -> ", batteryLevel);
            notifyClients(createJasonLetter("BAT",String(batteryLevel).c_str()));
        }    

    }

    if (qSecFlag)  // all 250 msec ... 
    {
        qSecFlag = FALSE;


    
        digitalWrite(TRIG_PIN, LOW);
        delay(5);
        digitalWrite(TRIG_PIN, HIGH);
        delay(5);
        digitalWrite(TRIG_PIN, LOW);

        distance = pulseIn(ECHO_PIN, HIGH);   // durch 58.23 

        // printf("distance : %d\n", (int)(distance/58.23));


        if ((int)(distance/58.23) < 12)
        {
            leds[2] = CRGB{255, 0, 255};
            leds[3] = CRGB{255, 0, 255};
        }
        else
        {
            leds[2] = CRGB{255, 255, 255};
            leds[3] = CRGB{255, 255, 255};
        }

        FastLED.show();
 


    }
    
    if (tenMSecFlag)
    {
        tenMSecFlag = FALSE;
    }

}    

void impuls_R_isr(void);
void impuls_L_isr(void);


void impuls_R_isr(void)
{
    leds[1] = CRGB{255, 0, 0};
 
}

void impuls_L_isr(void)
{
    leds[1] = CRGB{0, 255, 0};
  
}

//****************************************************************
// MPU6050 gyrometer:
//****************************************************************
//int getAngle(void)
//{
//    mpu6050.update();
//    return mpu6050.getAngleX();
//}



// Timer Interrupt: ################################################
// periodic timer interrupt, expires each 0.1 msec

void IRAM_ATTR myTimer(void)   
{
    static int32_t otick  = 0;
    static int32_t qtick = 0;
    static int32_t mtick = 0;
    static unsigned char ramp = 0;
    
    otick++;
    qtick++;
    mtick++;
    ramp++;

    if (otick >= WAIT_ONE_SEC) 
    {
        oneSecFlag = TRUE;
        otick = 0; 
    }

    if (qtick >= WAIT_250_MSEC) 
    {
        qSecFlag = TRUE;
        qtick = 0; 
    }

    if (mtick >= WAIT_10_MSEC) 
    {
        tenMSecFlag = TRUE;
        mtick = 0; 
    }

    // PWM:

    if (ramp >= vL) digitalWrite(WHEEL_L, LOW);  else digitalWrite(WHEEL_L, HIGH);
    if (ramp >= vR) digitalWrite(WHEEL_R, LOW);  else digitalWrite(WHEEL_R, HIGH);

}

/*


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);
}

void loop() {

  if(mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
/*
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Print out the values * /
    Serial.print("AccelX:");
    Serial.print(a.acceleration.x);
    Serial.print(",");
    Serial.print("AccelY:");
    Serial.print(a.acceleration.y);
    Serial.print(",");
    Serial.print("AccelZ:");
    Serial.print(a.acceleration.z);
    Serial.print(", ");
    Serial.print("GyroX:");
    Serial.print(g.gyro.x);
    Serial.print(",");
    Serial.print("GyroY:");
    Serial.print(g.gyro.y);
    Serial.print(",");
    Serial.print("GyroZ:");
    Serial.print(g.gyro.z);
    Serial.println("");
  }

  delay(10);
}


*/
