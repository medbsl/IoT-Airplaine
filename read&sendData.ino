
#include <STM32FreeRTOS.h>
#include <LPS25HBSensor.h>
#include <LSM6DSLSensor.h>
#include <SPI.h>
#include <WiFiST.h>
#include <PubSubClient.h>
#include <stdio.h> 
//#include <ArduinoJson.h>

// Includes.
#include <HTS221Sensor.h>
//char ssid[] = "ORANGE_C827";
//const char* password = "FM47FRK4W7UNX";

char ssid[] = "Monta";
//FM47FRK4W7UNX
const char* password = "xoxoxoxoxo";
/*String mqtt_server = "196.178.177.88";
char* mqqAddress;
mqqAddress= (char*) malloc(n* sizeof(char));


mqtt_server.toCharArray(mqqAddress,14);

*/

const char* mqtt_server = "broker.mqtt-dashboard.com";
SPIClass SPI_3(PC12, PC11, PC10);
WiFiClass WiFi(&SPI_3, PE0, PE1, PE8, PB13);
WiFiClient STClient;
int status = WL_IDLE_STATUS;     // the Wifi radio's status
char *MontaMQTT;
PubSubClient client(STClient);
long lastMsg = 0;
long lastMsg1 = 0;
long lastMsg2 = 0;
char msg[50];
long value = 0;
char accel[25];
char buffer_T[25];
char buffer_H[25];
char buffer_T2[25];
char buffer_P[25];

#define I2C2_SCL    PB10
#define I2C2_SDA    PB11

// Components.
HTS221Sensor  *HumTemp;
TwoWire *dev_i2c;

#if defined(ARDUINO_SAM_DUE)
#define DEV_I2C Wire1   //Define which I2C bus is used. Wire1 for the Arduino Due
#define SerialPort Serial
#else
#define DEV_I2C Wire    //Or Wire
#define SerialPort Serial
#endif


#define SerialPort Serial
#define I2C2_SCL    PB10
#define I2C2_SDA    PB11
LPS25HBSensor  *PressTemp;
LSM6DSLSensor *AccGyr;



void TaskTempreture( void *pvParameters );
void TaskGero( void *pvParameters );
//void TaskTemperaturePressure( void *pvParameters );



// the setup function runs once when you press reset or power the board
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
  // initialize serial communication at 9600 bits per second:
 
   DEV_I2C.begin();

  // Initlialize components.
  PressTemp = new LPS25HBSensor (&DEV_I2C);
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
    TaskTempreture
    ,  (const portCHAR *)"Tempreture"   // A name just for humans
    ,  256  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

    /*
    xTaskCreate(
    TaskTemperaturePressure
    ,  (const portCHAR *)"TaskTemperaturePressure"   // A name just for humans
    ,  256  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
*/
  xTaskCreate(
    TaskGero
    ,  (const portCHAR *) "GeroTask"
    ,  128  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL );

  // start scheduler
  vTaskStartScheduler();
  Serial.println("Insufficient RAM");
  while(1);
}


/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskTempreture(void *pvParameters)  // This is a task.
{
  (void) pvParameters;



  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
  // Read humidity and temperature.
  //int32_t humidity, temperature;
  float humidity, temperature;

  HumTemp->GetHumidity(&humidity);
  HumTemp->GetTemperature(&temperature);

  //dtostrf(humidity, 4, 2, buf);
  //sprintf(humid,"%s F", buf);

  // Output data.
  /*SerialPort.println("");
  Serial.print("Hum[%]: ");
  Serial.print(humidity, 2);
  Serial.print(" | Temp[C]: ");
  Serial.println(temperature, 2);*/
  /*
    ++value;
    snprintf (msg, 50, "hello world #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
      */

   long now1 = millis();
  if (now1 - lastMsg1 > 2000) {
    lastMsg1 = now1;
    ++value;
    snprintf (msg, 50, "hello world #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    dtostrf(temperature,5, 2, buffer_T);
    dtostrf(humidity,5, 2, buffer_H);


    //snprintf (buffer_TH, 50, "%ld %1d %1d %ld %ld %ld", accelerometer[0],accelerometer[1],accelerometer[2],gyroscope[0],gyroscope[1],gyroscope[2]);
    client.publish("3AGE2Temperature",buffer_T);
    client.publish("3AGE2Humidity",buffer_H);


    //snprintf (gyro, 50, "%ld %1d %1d", gyroscope[0],gyroscope[1],gyroscope[2]);
    //client.publish("3AGE2Gyro",gyro);
  }
  

      

  vTaskDelay( 100 ); // wait for one second
  }
}

void TaskGero(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {

    // read the input on analog pin 0:
    int32_t accelerometer[3];
  int32_t gyroscope[3];
  AccGyr->Get_X_Axes(accelerometer);
  AccGyr->Get_G_Axes(gyroscope);
  // Output data.
  /*SerialPort.println("");
  SerialPort.print("Acc[mg]: ");
  SerialPort.print(accelerometer[0]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer[1]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer[2]);
  SerialPort.print(" | Gyr[mdps]: ");
  SerialPort.print(gyroscope[0]);
  SerialPort.print(" ");
  SerialPort.print(gyroscope[1]);
  SerialPort.print(" ");
  SerialPort.println(gyroscope[2]);
  */
  if (!client.connected()) {
    reconnect();
  }
  //client.loop();
  // uncomment this 16h11

  
  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    snprintf (msg, 50, "hello world #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    
    
    snprintf (accel, 50, "%ld %1d %1d %ld %ld %ld", accelerometer[0],accelerometer[1],accelerometer[2],gyroscope[0],gyroscope[1],gyroscope[2]);
    client.publish("3AGE2Accel",accel);

    
  }
  
    vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
  }
}

/*
void TaskTemperaturePressure(void *pvParameters)  // This is a task.
{
  (void) pvParameters;



  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
  // Read humidity and temperature.
  //int32_t humidity, temperature;
  float pressure, temperature2;
  PressTemp->GetPressure(&pressure);
  PressTemp->GetTemperature(&temperature2);
 SerialPort.print("| Pres[hPa]: ");
  SerialPort.print(pressure, 2);
  SerialPort.print(" | Temp[C]: ");
  SerialPort.print(temperature2, 2);
  SerialPort.println(" |");
  

   long now2 = millis();
  if (now2 - lastMsg2 > 2000) {
    lastMsg2 = now2;
    ++value;
    snprintf (msg, 50, "hello world #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    dtostrf(temperature2,5, 2, buffer_T2);
    dtostrf(pressure,5, 2, buffer_P);


    //snprintf (buffer_TH, 50, "%ld %1d %1d %ld %ld %ld", accelerometer[0],accelerometer[1],accelerometer[2],gyroscope[0],gyroscope[1],gyroscope[2]);
    client.publish("3AGE2Temperature2",buffer_T2);
    client.publish("3AGE2Pressure",buffer_P);

  }

  vTaskDelay( 100 ); // wait for one second
  }

}
*/









void loop(){}


void setup_wifi() {

  delay(10);

  // initialize the WiFi module:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi module not detected");
    // don't continue:
    while (true);
  }

  // print firmware version:
  String fv = WiFi.firmwareVersion();
  Serial.print("Firmware version: ");
  Serial.println(fv);

  if (fv != "C3.5.2.3.BETA9") {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to network: ");
  Serial.println(ssid);
  while (status != WL_CONNECTED) {
    Serial.print(".");
    // Connect to WPA2 network:
    status = WiFi.begin(ssid, password);
    if (status != WL_CONNECTED) {
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

void callback(char* topic, byte* payload, unsigned int length) {
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

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("B-L475E-IOT01AClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("3AGE2", "{\"temperature\":10,\"humidity\":11,\"position\":12}");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
