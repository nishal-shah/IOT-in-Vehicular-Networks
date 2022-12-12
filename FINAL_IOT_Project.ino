#include "AdafruitIO_WiFi.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "DHT.h"
#include <LiquidCrystal_I2C.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include "secrets.h"
#include <DHT.h>

int lcdColumns = 16;
int lcdRows = 2;
// final dht11
#define DHTPIN 2     
#define DHTTYPE DHT11  
#define LED 16
#define WIFI_SSID       "OnePlus Nord"
#define WIFI_PASS       "5bef01858733"
#define IO_USERNAME "Mann_Shah"
#define IO_KEY "aio_TBnK502OR1FxnCrxu1R7IP3x3D0x"

float xi=-50000;
float yi=-50000;
float zi=-50000;
float t;
float xa;
float ya;
float za;

DHT dht(DHTPIN, DHTTYPE);

#define IO_USERNAME  "Mann_Shah"
#define IO_KEY       "aio_TBnK502OR1FxnCrxu1R7IP3x3D0x"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
AdafruitIO_Feed *temperatureFeed = io.feed("Temperature");
AdafruitIO_Feed *xaccFeed = io.feed("X-Acceleration");
AdafruitIO_Feed *yaccFeed = io.feed("Y-Acceleration");
AdafruitIO_Feed *zaccFeed = io.feed("Z-Acceleration");


LiquidCrystal_I2C lcd(0x3F, lcdColumns, lcdRows);  
 
/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
unsigned long lastMillis = 0;
unsigned long previousMillis = 0;
const long interval = 5000;
#define AWS_IOT_PUBLISH_TOPIC   "esp8266/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp8266/sub"
 
WiFiClientSecure net;
 
BearSSL::X509List cert(cacert);
BearSSL::X509List client_crt(client_cert);
BearSSL::PrivateKey key(privkey);
 
PubSubClient client(net);
 
time_t now;
time_t nowish = 1510592825;
 
 
void NTPConnect(void)
{
  Serial.print("Setting time using SNTP");
  configTime(TIME_ZONE * 3600, 0 * 3600, "asia.pool.ntp.org", "time.nist.gov");
  now = time(nullptr);
  while (now < nowish)
  {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("done!");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: ");
  Serial.print(asctime(&timeinfo));
}


 
void messageReceived(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Received [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
 
 
void connectAWS(void)
{
  delay(3000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 
  Serial.println(String("Attempting to connect to SSID: ") + String(WIFI_SSID));
 
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
 
  NTPConnect();
 
  net.setTrustAnchors(&cert);
  net.setClientRSACert(&client_crt, &key);
 
  client.setServer(MQTT_HOST, 8883);
  client.setCallback(messageReceived);
 
 
  Serial.println("Connecting to AWS IOT");
 
  while (!client.connect(THINGNAME))
  {
    Serial.print(".");
    delay(1000);
  }
 
  if (!client.connected()) {
    Serial.println("AWS IoT Timeout!");
    return;
  }
  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
 
  Serial.println("AWS IoT Connected!");
}
 
 
void publishMessage(void)
{
  StaticJsonDocument<200> doc;
  doc["time"] = millis();
  doc["temperature"] = t;
  doc["x_acceleration"] = xa;
  doc["y_acceleration"] = ya;
  doc["z_acceleration"] = za;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client
 
  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}
 
 
void displaySensorDetails(void)
{
 sensor_t sensor;
 accel.getSensor(&sensor);
 Serial.println("------------------------------------");
 Serial.print ("Sensor: "); Serial.println(sensor.name);
 Serial.print ("Driver Ver: "); Serial.println(sensor.version);
 Serial.print ("Unique ID: "); Serial.println(sensor.sensor_id);
 Serial.print ("Max Value: "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
 Serial.print ("Min Value: "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
 Serial.print ("Resolution: "); Serial.print(sensor.resolution); Serial.println(" m/s^2"); 
 Serial.println("------------------------------------");
 Serial.println("");
}
 
void displayDataRate(void)
{
 Serial.print ("Data Rate: "); 
 
 switch(accel.getDataRate())
 {
 case ADXL345_DATARATE_3200_HZ:
 Serial.print ("3200 "); 
 break;
 case ADXL345_DATARATE_1600_HZ:
 Serial.print ("1600 "); 
 break;
 case ADXL345_DATARATE_800_HZ:
 Serial.print ("800 "); 
 break;
 case ADXL345_DATARATE_400_HZ:
 Serial.print ("400 "); 
 break;
 case ADXL345_DATARATE_200_HZ:
 Serial.print ("200 "); 
 break;
 case ADXL345_DATARATE_100_HZ:
 Serial.print ("100 "); 
 break;
 case ADXL345_DATARATE_50_HZ:
 Serial.print ("50 "); 
 break;
 case ADXL345_DATARATE_25_HZ:
 Serial.print ("25 "); 
 break;
 case ADXL345_DATARATE_12_5_HZ:
 Serial.print ("12.5 "); 
 break;
 case ADXL345_DATARATE_6_25HZ:
 Serial.print ("6.25 "); 
 break;
 case ADXL345_DATARATE_3_13_HZ:
 Serial.print ("3.13 "); 
 break;
 case ADXL345_DATARATE_1_56_HZ:
 Serial.print ("1.56 "); 
 break;
 case ADXL345_DATARATE_0_78_HZ:
 Serial.print ("0.78 "); 
 break;
 case ADXL345_DATARATE_0_39_HZ:
 Serial.print ("0.39 "); 
 break;
 case ADXL345_DATARATE_0_20_HZ:
 Serial.print ("0.20 "); 
 break;
 case ADXL345_DATARATE_0_10_HZ:
 Serial.print ("0.10 "); 
 break;
 default:
 Serial.print ("???? "); 
 break;
 } 
 Serial.println(" Hz"); 
}
 
void displayRange(void)
{
 Serial.print ("Range: +/- "); 
 
 switch(accel.getRange())
 {
 case ADXL345_RANGE_16_G:
 Serial.print ("16 "); 
 break;
 case ADXL345_RANGE_8_G:
 Serial.print ("8 "); 
 break;
 case ADXL345_RANGE_4_G:
 Serial.print ("4 "); 
 break;
 case ADXL345_RANGE_2_G:
 Serial.print ("2 "); 
 break;
 default:
 Serial.print ("?? "); 
 break;
 } 
 Serial.println(" g"); 
}

void welcome(void){
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("Welcome");
   lcd.setCursor(0,1);
   lcd.print("Vinay Sir!");
   delay(3000);
   lcd.clear();
} 

void setup(void) 
{ 
 // connect();

  pinMode(LED,OUTPUT);
  Wire.begin(12,14);
 // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();
  Serial.begin(9600);
  io.connect();
  Serial.setTimeout(2000);
  Serial.print("Reached 1");
  // Wait for serial to initialize.
  while(!Serial) { }
  
  dht.begin();

  Serial.println("Device Started");
  Serial.println("-------------------------------------");
  Serial.println("Running DHT!");
  Serial.println("-------------------------------------");
  Serial.println("Accelerometer Test"); Serial.println("");
 
 /* Initialise the sensor */
 if(!accel.begin())
 {
 /* There was a problem detecting the ADXL345 ... check your connections */
 Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
 while(1);
 }
 
 /* Set the range to whatever is appropriate for your project */
 accel.setRange(ADXL345_RANGE_16_G);
 // displaySetRange(ADXL345_RANGE_8_G);
 // displaySetRange(ADXL345_RANGE_4_G);
 // displaySetRange(ADXL345_RANGE_2_G);
 
 /* Display some basic information on this sensor */
 displaySensorDetails();
 
 /* Display additional settings (outside the scope of sensor_t) */
 displayDataRate();
 displayRange();
 Serial.println("");
 connectAWS();

 while(io.status() < AIO_CONNECTED) 
  {
    Serial.print(".");
    delay(500);
  }
 welcome();
}

int timeSinceLastRead = 0;

void loop(void) 
{
  io.run();
  
 /* Get a new sensor event */ 
 sensors_event_t event; 
 accel.getEvent(&event);


  
 // Report every 2 seconds.
  if(timeSinceLastRead > 2000) {
     t = dht.readTemperature();
    float f = dht.readTemperature(true);
    // Check if any reads failed and exit early (to try again).
    if (isnan(t) || isnan(f)) {
      Serial.println("Failed to read from DHT sensor!");
      timeSinceLastRead = 0;
      return;
    }

    if(t>=25)
    {
      digitalWrite(LED,HIGH);
    }
    else
    {
      digitalWrite(LED,LOW);
    }

    temperatureFeed->save(t);
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.println(" *C ");
     /* Display the results (acceleration is measured in m/s^2) */
 if(xi==-50000 || yi==-50000 || zi==-50000)
 {
  xi=event.acceleration.x;
  yi=event.acceleration.y;
  zi=event.acceleration.z;
 }
 xa=event.acceleration.x-xi;
 ya=event.acceleration.y-yi;
 za=event.acceleration.z-zi;
 xaccFeed->save(xa);
 yaccFeed->save(ya);
 zaccFeed->save(za);
 
 Serial.print("X: "); Serial.print(event.acceleration.x-xi); Serial.print(" ");
 Serial.print("Y: "); Serial.print(event.acceleration.y-yi); Serial.print(" ");
 Serial.print("Z: "); Serial.print(event.acceleration.z-zi); Serial.print(" ");Serial.println("m/s^2 ");

  lcd.setCursor(0, 0);
  lcd.print("Temperature:");
  lcd.setCursor(0,1);
  lcd.print(t);lcd.print(" *C ");
  delay(5000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("X:"); lcd.print(event.acceleration.x-xi); lcd.print(" ");
  lcd.print("Y:"); lcd.print(event.acceleration.y-yi); lcd.print(" ");
  lcd.setCursor(0,1);
  lcd.print("Z:"); lcd.print(event.acceleration.z-zi); lcd.print(" ");lcd.println("m/s^2   ");
  delay(5000);
  lcd.clear();

    timeSinceLastRead = 0;
  }
  delay(100);
  timeSinceLastRead += 100;
  now = time(nullptr);
 
  if (!client.connected())
  {
    connectAWS();
  }
  else
  {
    client.loop();
    if (millis() - lastMillis > 5000)
    {
      lastMillis = millis();
      publishMessage();
    }
  }
}
