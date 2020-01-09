/**============================ (c) 3AGE2 2019-2020 ============================ 
 **  File Name   :   STM32L475_FreeRTOS_IOT.c
 **  Created on  :   Jan 2, 2020
 **  Author      :   Montassar BOUARGOUB & Mohamed BOUSSELMI

 **                            CONFIDENTIAL
 **-----------------------------------------------------------------------------
 **  File Description: 

 **   An application for the STMicroelectronics STM32L475 Discovery Kit IoT Node using
 **   ARM Mbed OS to connect to MQTT Broker.

 **   This application is decicated to aeronautic field, such as airplanes. The main 
 **   idea of the project is to supervize the current state (Temperature, Humidity, 
 **   Pressure, Gyroscope, Accelerometer) of different air planes in real time.

 **   The real-time data will be displayed in a cloud dashboard. When a plane
 **   faces an unnatural conditions, an notification will be sent to the user to 
 **   notify him about the news.
 **===========================================================================*/
/**=============================================================================
 **                                 Includes
 **===========================================================================*/
#include <STM32FreeRTOS.h>
#include <LPS22HBSensor.h>
#include <LSM6DSLSensor.h>
#include <SPI.h>
#include <WiFiST.h>
#include <PubSubClient.h>
#include <stdio.h> 
#include <HTS221Sensor.h>

/**=============================================================================
 **                              Defines / Constants
 **===========================================================================*/

#define I2C2_SCL    PB10
#define I2C2_SDA    PB11


#if defined(ARDUINO_SAM_DUE)
#define DEV_I2C Wire1   //Define which I2C bus is used. Wire1 for the Arduino Due
#define SerialPort Serial
#else
#define DEV_I2C Wire    
#define SerialPort Serial
#endif

#define SerialPort Serial
#define I2C2_SCL    PB10
#define I2C2_SDA    PB11


/**=============================================================================
 **                             Global Variables
 **===========================================================================*/

char ssid[] = "ORANGE_C827";
const char* password = "2LKFZXNQ";

const char* mqtt_server = "broker.mqtt-dashboard.com";
SPIClass SPI_3(PC12, PC11, PC10);
WiFiClass WiFi(&SPI_3, PE0, PE1, PE8, PB13);
WiFiClient STClient;
int status = WL_IDLE_STATUS;      // the Wifi radio's status
PubSubClient client(STClient);
long PreviousTime2 = 0;            
long PreviousTime1 = 0;
char msg[50];
long NumberOfMessagesPublished = 0;
char accel[25];
char buffer_T[25];                // Temperature buffer
char buffer_H[25];                // Humidity buffer
char buffer_T2[25];               // Temperature2 buffer
char buffer_P[25];                // Pressure buffer

// Components declarations
HTS221Sensor  *HumTemp;

LPS22HBSensor *PressTemp;

LSM6DSLSensor *AccGyr;

TwoWire *dev_i2c;

/**=============================================================================
 **                         Private Function Prototypes
 **===========================================================================*/

void TaskTemperatureHumidity( void *pvParameters );
void TaskGyroAcceleroTempPressure( void *pvParameters );

// Setup Wifi function to connect to the wireless wifi

void setup_wifi() 
{
  delay(10);

  // initialize the WiFi module:
  if (WiFi.status() == WL_NO_SHIELD) 
  {
    
    Serial.println("WiFi module not detected");
    // don't continue:
    while (true);
  }

  // print firmware version:
  String fv = WiFi.firmwareVersion();
  Serial.print("Firmware version: ");
  Serial.println(fv);

  if (fv != "C3.5.2.3.BETA9") 
  {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to network: ");
  Serial.println(ssid);
  while (status != WL_CONNECTED) 
  {

    Serial.print(".");
    // Connect to WPA2 network:
    status = WiFi.begin(ssid, password);
    if (status != WL_CONNECTED) 
    {
      // Connect to WPA (TKIP) network:
      status = WiFi.begin(ssid, password, ES_WIFI_SEC_WPA);
    }
    // wait 10 seconds for connection:
    delay(10000);
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) 
{

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(LED_BUILTIN, HIGH);   // Turn the LED on
  } else {
    digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off
  }
}

// Reconnect function is called when a MQTT error occurs

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("B-L475E-IOT01AClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("TestingConnection", "Connected");
      // ... and resubscribe
      client.subscribe("TestingConnection");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


// The setup function runs once when you press reset or power the board
void setup() {

  // Initialize serial for output.
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  // Initialize I2C bus.   Communication between the sensors and CPU
  dev_i2c = new TwoWire(I2C2_SDA, I2C2_SCL);
  dev_i2c->begin();

  // Initlialize components.
  HumTemp = new HTS221Sensor (dev_i2c);
  HumTemp->Enable();
  // Initialize serial communication at 9600 bits per second:
 
   DEV_I2C.begin();

   // Initlialize components.
  PressTemp = new LPS22HBSensor(dev_i2c);
  PressTemp->Enable();

  // Initlialize components.
  AccGyr = new LSM6DSLSensor(dev_i2c, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW);
  AccGyr->Enable_X();
  AccGyr->Enable_G();

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskTemperatureHumidity
    ,  (const portCHAR *)"TemperatureHumidity"   // Task's name
    ,  256  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

    
  xTaskCreate(
    TaskGyroAcceleroTempPressure
    ,  (const portCHAR *) "GyroAcceleroTempPressure"     // Task's name
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  // Start scheduler
  vTaskStartScheduler();

  while(1);
}


/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

// This task is used to get Temperature and Humidity data from 
// sensors and send it to MQTT Broker

void TaskTemperatureHumidity(void *pvParameters)  
{

  (void) pvParameters;

  // Initialize digital LED_BUILTIN on pin 13 as an output.

  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {

    // Read humidity and temperature.
    float humidity, temperature;

    HumTemp->GetHumidity(&humidity);
    HumTemp->GetTemperature(&temperature);

    long Timer1 = millis();
    
    if (Timer1 - PreviousTime1 > 2000) 
    {
      PreviousTime1 = Timer1;
      ++NumberOfMessagesPublished;
      snprintf (msg, 50, "Published message number:  #%ld", NumberOfMessagesPublished);
      Serial.print("Publish message: ");
      Serial.println(msg);
      dtostrf(temperature,5, 2, buffer_T);
      dtostrf(humidity,5, 2, buffer_H);

      client.publish("3AGE2Temperature",buffer_T);
      client.publish("3AGE2Humidity",buffer_H);

    }
    
    vTaskDelay( 100 ); // wait for 100 ms
  
  }

}

// This task is used to get Gyroscope, Accelerometer, Temperature and Pressure
// data from sensors and send it to MQTT Broker

void TaskGyroAcceleroTempPressure(void *pvParameters)  // This is a task.
{

  (void) pvParameters;

  for (;;)
  {

    // read the input on analog pin 0:

    int32_t accelerometer[3];
    int32_t gyroscope[3];
    AccGyr->Get_X_Axes(accelerometer);
    AccGyr->Get_G_Axes(gyroscope);

    // Pressure and temperature

    float pressure, temperature2;
    PressTemp->GetPressure(&pressure);
    PressTemp->GetTemperature(&temperature2);

    if (!client.connected()) {
      reconnect();
    }
       
    long Timer2 = millis();
    if (Timer2 - PreviousTime2 > 2000) 
    {

      PreviousTime2 = Timer2;
      ++NumberOfMessagesPublished;
      snprintf (msg, 50, "Published message number:  #%ld", NumberOfMessagesPublished);
      Serial.print("Publish message: ");
      Serial.println(msg);

      // Pressure and temperature

      dtostrf(temperature2,5, 2, buffer_T2);
      dtostrf(pressure,5, 2, buffer_P);

      client.publish("3AGE2Temperature2",buffer_T2);
      client.publish("3AGE2Pressure",buffer_P);
      
      snprintf (accel, 50, "%ld %1d %1d %ld %ld %ld", accelerometer[0],accelerometer[1],accelerometer[2],gyroscope[0],gyroscope[1],gyroscope[2]);
      client.publish("3AGE2Accel",accel);

    }
    
      vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
  }

}


void loop(){}
