 /**
 *     PROJECT: MySensors / Small battery sensor low power 8 mhz
 *     PROGRAMMER: Jumping
 *     DATE: october 10, 2016/ last update: august 10, 2021
 *     FILE: sensor17_2021_RaimUV_CEECH.ino
 *     LICENSE: Public domain
 *    
 *     Hardware: ATMega328p board w/ RGM69chw
 *        and MySensors 2.3
 *            
 *    Special:
 *        program with Arduino Pro 3.3V 8Mhz!!!
 *        
 *    Summary:
 *        low power (battery)
 *        BH1750FVI Light sensor
 *        VEML6070
 *        voltage meter for battery
 *        Voltage meter for solar
 *        current charge
 *    Board:
 *      arduino pro / pro mini
 *      ATmega 328P
 *      external clock 8Mhz
 *      
 *    Remarks:
 *
 *  i2c connection for VEML6070 and BH1750

*******************************
 *
 * REVISION HISTORY
 * Version 1.0 - idefix
 * 
 * DESCRIPTION
 * Arduino BH1750FVI Light sensor
 * communicate using I2C Protocol
 * this library enable 2 slave device addresses
 * Main address  0x23
 * secondary address 0x5C
 * connect the sensor as follows :
 *
 *   VCC  >>> 5V
 *   Gnd  >>> Gnd
 *   ADDR >>> NC or GND  
 *   SCL  >>> A5
 *   SDA  >>> A4
 * http://www.mysensors.org/build/light
 * 
 * USING CEECH BOARD
 */
// Enable debug prints to serial monitor
#define MY_DEBUG 
#define MY_RADIO_RFM69
#define MY_RFM69_FREQUENCY RFM69_868MHZ
#define MY_RFM69_NEW_DRIVER
#define MY_IS_RFM69HW
#define MY_RFM69_MAX_POWER_LEVEL_DBM (10)
#define MY_RFM69_CS_PIN 8
//--
#define MY_TRANSPORT_WAIT_READY_MS 10000
#define MY_SIGNAL_REPORT_ENABLED
// NODE ID
#define MY_NODE_ID 17
#define NODE_TXT "RAIN"  // Text to add to sensor name
#define WAIT_TIME 50
//
//#define MY_SIGNING_SOFT
//#define MY_SIGNING_SOFT_RANDOMSEED_PIN 17 //!< A3 - pin where ATSHA204 is attached
//#define MY_SIGNING_REQUEST_SIGNATURES
//#define MY_SIGNING_SIMPLE_PASSWD "gianpi130318"
#include <MySensors.h>
#include <Adafruit_VEML6070.h>
#include <hp_BH1750.h>  //include the library
#include <Wire.h>
////-------------------------------------------------------------------
#define LTC4079_CHRG_PIN            A7      //analog input A7 on ATmega 328 is /CHRG signal from LTC4079
#define batteryVoltage_PIN          A0      //analog input A0 on ATmega328 is battery voltage
#define solarVoltage_PIN            A2      //analog input A2 is solar cell voltage
#define chargeCurrent_PIN           A6      //analog input A6 is battery charge current
//-------------------------------------------------------------------
#define tipSensor_PIN               3       // The reed switch you attached.  (Only 2 and 3 generates interrupt!)
#define INTERRUPT tipSensor_PIN-2           // Usually the interrupt = pin -2 (on uno/nano anyway)
//-------------------------------------------------------------------
// Reference values for ADC and Battery measurements
const float VccMin          = 1.0*2.8 ;             // Minimum expected Vcc level, in Volts.
const float VccMax          = 1.0*4.2 ;             // Maximum expected Vcc level, in Volts. 
const float VccCorrection   = 3.32/3.3 ;            // Measured Vcc by multimeter divided by reported Vcc
//float VccReference = 4.3 ;                          // voltage reference for measurement, definitive init in setup
//-------------------------------------------------------------------
#define LIGHT_CHILD_ID      1
#define UV_CHILD_ID         2
#define RAIN_CHILD_ID       3  // Keeps track of accumulated rainfall
#define BATT_CHILD_ID       7
#define RSSI_CHILD_ID       8
//-------------------------------------------------------------------
  const float uvThreshold = 10 ;                    // send only if change > treshold
  const float rssiThreshold = 1 ;
  const float battThreshold = 0.1 ;
  const uint8_t heartbeat = 6 ;                       // heartbeat every hour (x times SLEEP_TIME)
  unsigned long lastHeartbeat = 0 ;
  const unsigned long SLEEP_TIME = 600000UL;      // 10 min Sleep time between reads (in milliseconds)
//-------------------------------------------------------------------
  float     lastUV = -1;
  uint16_t  lastLux = -1;
  float     lastBattVoltage = -1;
  float     lastSolarVoltage = -1;
  float     lastBattCurrent = -1;
  uint8_t   lastBattPct = 0;
  int       lastRSSI = 0;
  // flags to indicate if transmission is needed, heartbeat and/or changes > treshold
  boolean txLux = true ;
  boolean txUV = true ;
  boolean txRain = true ;  
  boolean txBattVoltage = true ;
  boolean txSolarVoltage = true ;
  boolean txBattCurrent = true ;
  boolean txRSSI = true ;
//-------------------------------------------------------------------
  float hwRainVolume = 0;                         // Current rainvolume calculated in hardware.
  unsigned long hwPulseCounter = 0;               // Pulsecount recieved from GW
  float fullCounter = 0;                          // Counts when to send counter
  float bucketSize = 0.2794;                      // Bucketsize mm, needs to be 1, 0.5, 0.25, 0.2 or 0.1
  boolean pcReceived = false;                     // If we have recieved the pulscount from GW or not 
  boolean reedState;                              // Current state the reedswitch is in
  boolean oldReedState;                           // Old state (last state) of the reedswitch
  unsigned long lastSend = 0;                     // Time we last tried to fetch counter.
//-------------------------------------------------------------------
  const char *uvindex[] = { "0-2 Low", "3-5 Moderate", "6-7 High", "8-10 Very high", ">11 Extreme", "Unknown" };
  enum UV_INDEX
  {
    LOW_LEVEL = 0,                     
    MODERATE_LEVEL = 1,                
    HIGH_LEVEL = 2,                    
    VERY_HIGH_LEVEL = 3,                 
    EXTREME_LEVEL = 4,                 
    UNKNOWN = 5
  };
//-------------------------------------------------------------------
  uint8_t uvi;
//-------------------------------------------------------------------
  //BH1750 lightSensor;                                       // lux meter
  hp_BH1750 lightSensor;                                      //create the sensor object
  Adafruit_VEML6070 uv = Adafruit_VEML6070();                 // UV Meter
//-------------------------------------------------------------------
  MyMessage msgLux(LIGHT_CHILD_ID, V_LEVEL);                  // BH 1750 Light sensor (lux)
  MyMessage msgUVI(UV_CHILD_ID, V_UV);
  MyMessage msgUV(UV_CHILD_ID, V_VAR2);                       // VEML6070 UV Sensor
  MyMessage msgRain(RAIN_CHILD_ID, V_RAIN);                   // Bucket sensor
  MyMessage msgLastCounter(RAIN_CHILD_ID,V_VAR1);
//--------------------------------------------------------------------------------
  MyMessage msgBatteryVoltage(BATT_CHILD_ID, V_VOLTAGE);      // Battery voltage (V)
  MyMessage msgSolarVoltage(BATT_CHILD_ID, V_VAR3);           // Solar voltage (V)
  MyMessage msgBatteryCurrent(BATT_CHILD_ID, V_VAR4);         // Battery current (mA)
  MyMessage msgRSSI(RSSI_CHILD_ID, V_LEVEL);                  // RSSI
//--------------------------------------------------------------------------------
void setup()  
{ 
  Serial.begin(115200);
  Wire.begin();                                      // START I2C
  //lightSensor.begin();                             // LIGHT SENSOR
  bool avail = lightSensor.begin(BH1750_TO_GROUND);  // use BH1750_TO_GROUND or BH1750_TO_VCC depending how you wired the address pin of the sensor.
  uv.begin(VEML6070_2_T);                            // UV SENSOR
  //-------------------------  
  pinMode(tipSensor_PIN, INPUT); //HW debouncing
  reedState = digitalRead(tipSensor_PIN); // Read what state the reedswitch is in
  oldReedState = reedState; // Set startup position for reedswitch 
  //-------------------------
  Serial.println("Startup completed");
}
//-------------------------------------------------------------------
void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("JMP "NODE_TXT, "1.0");
  wait(WAIT_TIME);
  // Register all sensors to gateway (they will be created as child devices)
  present(BATT_CHILD_ID, S_MULTIMETER, "Batt " NODE_TXT);   // Battery parameters
  wait(WAIT_TIME);
  present(LIGHT_CHILD_ID, S_LIGHT_LEVEL,"Light " NODE_TXT); // Light sensor
  wait(WAIT_TIME);
  present(UV_CHILD_ID, S_UV, "UV " NODE_TXT);               // UV
  wait(WAIT_TIME);
  present(RAIN_CHILD_ID, S_RAIN, "Rain " NODE_TXT);         // RAIN Sensor
  wait(WAIT_TIME);
  present(RSSI_CHILD_ID, S_SOUND, "RSSI " NODE_TXT);        // RSSI Sensor
}
//-------------------------------------------------------------------
void loop(){
  //Serial.println("LOOP initialized");
  unsigned long currentTime = millis();
  //See if we have the counter/pulse from HA - and ask for it if we dont.
  if (!pcReceived && (currentTime - lastSend > 5000)) {      
    request(RAIN_CHILD_ID, V_VAR1);
    lastSend=currentTime;
    return;
    }
  if (!pcReceived) {
    return;
    }   
  //Read if the bucket tipped over
  reedState = digitalRead(tipSensor_PIN);
  boolean tipped = oldReedState != reedState;
  //BUCKET TIPS!
  if (tipped==true) {
    Serial.println("The bucket has tipped over...");
    oldReedState = reedState;
    hwRainVolume = hwRainVolume + bucketSize;
    send(msgRain.set((float)hwRainVolume,1));
    wait(1000);
    fullCounter = fullCounter + bucketSize;
    //Count so we send the counter for every 1mm
    if(fullCounter >= 1){
      hwPulseCounter++;
      send(msgLastCounter.set(hwPulseCounter));
      wait(1000);
      fullCounter = 0;
      }
  }
  if (tipped==false) {
    Serial.println("TIPPED FALSE...");
    readLux();
    readUV();
    readRSSI();
    readVoltages();
    sendSensors();
    }
  //-------------------------
  lastSend=currentTime;
  Serial.println("Go to sleep");
  sleep(INTERRUPT, CHANGE, SLEEP_TIME); //The interupt can be CHANGE or FALLING depending on how you wired the hardware.
}
//------------------------------------------------------------------

//Read if we have a incoming message.
void receive(const MyMessage &msg) {
    if (msg.type==V_VAR1) {
    hwPulseCounter = msg.getULong();
    hwRainVolume = hwPulseCounter;
    pcReceived = true;
    #ifdef MY_DEBUG
      Serial.print("Received last pulse count from gw: ");
      Serial.println(hwPulseCounter);
    #endif       
    }
}

//-------------------------------------------------------------------
void readRSSI(void){ 
    int receivingRSSI;
    receivingRSSI = transportGetReceivingRSSI();  // read RSSI in RFM69. Measure reception signal from gw
    if (abs(receivingRSSI  - lastRSSI) >= rssiThreshold) {
      lastRSSI = receivingRSSI;
      txRSSI= true;
      } 
    #ifdef MY_DEBUG
     Serial.print("receiving RSSI: ");
     Serial.print(receivingRSSI);
     Serial.println("db");
    #endif     
}
//-------------------------------------------------------------------
void readLux(void){
   //uint16_t lux = lightSensor.readLightLevel();             //Get Lux value
   lightSensor.start();   //starts a measurement
   float lux=lightSensor.getLux();
   if (abs(lux  - lastLux) >= uvThreshold) {
      lastLux = lux;
      txLux= true;
   }   
   #ifdef MY_DEBUG
      Serial.print("BH1750 lux: ");
      Serial.println(lux);
   #endif
}

//-------------------------------------------------------------------
void readUV(void) {
  int uvIntensity = uv.readUV();
  if (abs(uvIntensity  - lastUV) >= uvThreshold) {
      lastUV = uvIntensity;
      txUV=true;
  }  
  uvi = UNKNOWN;
  if (uvIntensity >= 4109) {
     uvi = EXTREME_LEVEL;
   }
  else if ((uvIntensity>=2989) && (uvIntensity < 4108)) {
    uvi = VERY_HIGH_LEVEL;
  }
  else if ((uvIntensity >=2242) && (uvIntensity < 2988)) {
    uvi = HIGH_LEVEL;
  }
  else if ((uvIntensity>= 1121) && (uvIntensity < 2241)) {
    uvi = MODERATE_LEVEL;
  }
  else if ((uvIntensity >= 0) && (uvIntensity < 1120)) {
    uvi = LOW_LEVEL;
  }
  else {
    uvi = UNKNOWN;
    }
  #ifdef MY_DEBUG
    Serial.print("UV Intensity: ");
    Serial.print(uvIntensity);
    Serial.println();
    Serial.print("UV Index: ");
    Serial.print(uvi);
    Serial.println();
    Serial.print("UV Index: ");
    Serial.print(uvindex[uvi]);
    Serial.println();
  #endif
}

//-------------------------------------------------------------------
void readVoltages(void)
{ 
   float voltage = readVcc(); // actual VOLTAGE VCC
   #ifdef MY_DEBUG
     Serial.print("Vcc ");
     Serial.print(voltage);
     Serial.println("V ");
   #endif
   //------------------
   float batteryChargeCurrent = ((float)analogRead(chargeCurrent_PIN) * voltage/1024)/ 3.3 * 250; // CURRENT battery
   if (abs(batteryChargeCurrent  - lastBattCurrent) >= battThreshold) {
      lastBattCurrent = batteryChargeCurrent;
      txBattCurrent= true;
   }   
   #ifdef MY_DEBUG
     Serial.print("Charge current ");
     Serial.print(batteryChargeCurrent);
     Serial.println("mA ");
   #endif
   //------------------
   float batteryVoltage = ((float)analogRead(batteryVoltage_PIN) * voltage/1024)* 2; // VOLTAGE battery
   if (abs(batteryVoltage  - lastBattVoltage) >= battThreshold) {
      lastBattVoltage = batteryVoltage;
      txBattVoltage= true;
   } 
   #ifdef MY_DEBUG
     Serial.print("Battery voltage ");
     Serial.print(batteryVoltage);
     Serial.println("V ");
   #endif
   //------------------
   float solarVoltage = ((float)analogRead(solarVoltage_PIN) * voltage/1024)* 2; // VOLTAGE solar
   if (abs(solarVoltage  - lastSolarVoltage) >= battThreshold) {
      lastSolarVoltage = solarVoltage;
      txSolarVoltage = true;
   } 
   #ifdef MY_DEBUG
     Serial.print("Solar voltage ");
     Serial.print(solarVoltage);
     Serial.println("V ");
   #endif
   //------------------
   int charge = (int)analogRead(LTC4079_CHRG_PIN);
   #ifdef MY_DEBUG
     Serial.print("CHRG ");
     Serial.println(charge);
   #endif
}
//-------------------------------------------------------------------
void sendSensors(void)
{
    lastHeartbeat++ ;                                       // update Heartbeatcount every call
    if ( lastHeartbeat > heartbeat) {                       // if heartbeat update all sensors & battery status
        txBattVoltage = txBattCurrent = txSolarVoltage = txUV = txLux = txRain = txRSSI = true ;
        lastHeartbeat = 0;
        }
    if (txUV){
        send(msgUV.set(lastUV, 2));                             // Send UV radiation
        send(msgUVI.set(uvindex[uvi]));                    // Send UV INDEX
        txUV = false;
        wait(WAIT_TIME);
        }
     if (txLux){
        send(msgLux.set(lastLux));                              // Send lux
        txLux = false ;
        wait(WAIT_TIME);
        }
    if (txRain) {                                               // Send Rain
        send(msgRain.set((float)hwRainVolume,2));
        wait(800);
        send(msgLastCounter.set(hwPulseCounter));
        txRain = false;
        wait(WAIT_TIME);
        }
    if (txRSSI){
        send(msgRSSI.set(lastRSSI));                  // send RSSI level
        txRSSI = false;
        wait(WAIT_TIME);
        }
    if (txBattVoltage){
        send(msgBatteryVoltage.set(lastBattVoltage, 2));        // Send battery V
        txBattVoltage = false;
        wait(WAIT_TIME);
        int batteryPcnt = 100.0*(lastBattVoltage - VccMin)/(VccMax - VccMin); // Convert voltage to percentage
        //int batteryPcnt = static_cast<int>(((lastVoltage-VccMin)/(VccMax - VccMin))*100.);
        #ifdef MY_DEBUG
           Serial.print("Battery: ");
           Serial.print(batteryPcnt);
           Serial.println(" %");
        #endif
        sendBatteryLevel(batteryPcnt);
        wait(WAIT_TIME);
        }
    if (txBattCurrent){
        send(msgBatteryCurrent.set(lastBattCurrent, 2));        // Send battery I
        txBattCurrent = false;
        wait(WAIT_TIME);
        }
    if (txSolarVoltage){
        send(msgSolarVoltage.set(lastSolarVoltage, 2));         // Send solar V
        txBattCurrent = false;
        wait(WAIT_TIME);
        }
     //------------------
}
//-------------------------------------------------------------------
float readVcc() 
{
  signed long resultVcc;
  float resultVccFloat;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(10);                           // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                 // Convert
  while (bit_is_set(ADCSRA,ADSC));
  resultVcc = ADCL;
  resultVcc |= ADCH<<8;
  resultVcc = 1126400L / resultVcc;    // Back-calculate AVcc in mV
  resultVccFloat = (float) resultVcc / 1000.0; // Convert to Float
  return resultVccFloat;
}
