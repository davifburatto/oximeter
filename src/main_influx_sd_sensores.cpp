#include <Wire.h>
#include <TFT_eSPI.h>
#include <OneButton.h>
#include <pcf8563.h>
#include "esp_adc_cal.h"
#include "image.h"
#include <max32664.h>
#include <SPI.h>
#include <ezTime.h>				 
#include "MPU6050.h"
#include "heartRate.h"
#include <SensirionI2CScd4x.h>
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#include <mySD.h>					  
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

/*INFLUXDB Config*/
// WiFi AP SSID
#define WIFI_SSID "BURATTO2"
// WiFi password
#define WIFI_PASSWORD "141213daviedrica"
// InfluxDB v2 server url, e.g. https://eu-central-1-1.aws.cloud2.influxdata.com (Use: InfluxDB UI -> Load Data -> Client Libraries)
#define INFLUXDB_URL "http://iot.buratto.eng.br:8086"
// InfluxDB v2 server or cloud API token (Use: InfluxDB UI -> Data -> API Tokens -> <select token>)
#define INFLUXDB_TOKEN "NI7r8G0Npudsb0ThM_S9Z4es1GymlIOpaCLIBZxN5ZGNUYdk54TvJCzQf5TMCPFbr9bFfa-U33H_5LiS6CXahA=="
// InfluxDB v2 organization id (Use: InfluxDB UI -> User -> About -> Common Ids )
#define INFLUXDB_ORG "PUC"
// InfluxDB v2 bucket name (Use: InfluxDB UI ->  Data -> Buckets)
#define INFLUXDB_BUCKET "sinais_vitais"
// Set timezone string according to https://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
// Examples:
#define TZ_INFO "<-03>3"

/*********************/


#define  I2C_SDA_PIN             21
#define  I2C_SCL_PIN             22
#define  RTC_INT_PIN             34
#define  BATT_ADC_PIN            35
#define  VBUS_PIN                37
#define  LED_PIN                 33
#define  CHARGE_PIN              32
#define  BUTTON_PIN              38
#define  MPU_INT                 39
#define  RESET_PIN               02
#define  MFIO_PIN                04
#define  RAWDATA_BUFFLEN         200

#define debug

//extern const int porcentagem;

TFT_eSPI    tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
PCF8563_Class rtc;
MPU6050     mpu;
OneButton   button(BUTTON_PIN, true);
max32664 MAX32664(RESET_PIN, MFIO_PIN, RAWDATA_BUFFLEN);

/*INFLUXDB Config*/
// InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// Data point
Point sensor("Monitor_1");
/*********************/

//sdcard
File myFile;
const int chipSelect = 5;

bool        nocal=0;
									 
bool        pass = 0;								   
char        buff[256];
bool        rtcIrq = false;
bool        initial = 1;
uint8_t     func_select = 0;
uint8_t     omm = 99;
uint8_t     xcolon = 0;
uint32_t    targetTime = 0;       // for next 1 second timeout
uint32_t    targetTime1 = 0;
uint32_t    targetTimescd = 0;
uint32_t    measureTime = 0;       // for next 100ms timeout
uint32_t    colour = 0;
int         vref = 1100;
//bool        charge_indication = false;
uint8_t     hh, mm, ss, dia, mes;
uint16_t    ano;

/*MAX30105*/
const uint8_t RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
uint8_t     rates[RATE_SIZE]; //Array of heart rates
uint8_t     rateSpot = 0;
long        lastBeat = 0; //Time at which the last beat occurred
float       beatsPerMinute;
int         beatAvg;
bool        find_max30105 = false;
bool        showError = false;
uint8_t    calib_lvl;
// Read Measurement
uint16_t co2;
float temperature;
float humidity;
float systol;
float diastol;
float heart;
float oxig;
String dataMessage;
String timelog;
unsigned long tseconds; 
unsigned long epochTime; 
// Timer BMP variables
unsigned long lastTimebmp = 0;
unsigned long timerDelaybmp = 100;//250
unsigned long lastTimesd = 0;
unsigned long timerDelaysd = 100;//250
unsigned long lastTimenuvem = 0;
unsigned long timerDelaynuvem = 100;//250
uint32_t    lasttimeSCD = 0;
uint32_t    timerDelaySCD = 5000;


SensirionI2CScd4x scd4x;
		  

void drawProgressBar(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint8_t percentage, uint16_t frameColor, uint16_t barColor);

void initSDCard(void);
void mfioInterruptHndlr(void);
void enableInterruptPin(void);
void loadAlgomodeParameters(void);
bool setupMAX30105(void);
void loopMAX30105(void);
bool setupWiFi(void);
void setupOTA(void);
void loopOTA(void);
//void setupMonitor();
bool setupRTC(void);
void clickHandle(void);
void setup(void);
String getVoltage(void);
void page1(void);


String get_date()
{   String get_date_time;
//get time
    RTC_Date datetime = rtc.getDateTime();
    hh = datetime.hour;
    mm = datetime.minute;
    ss = datetime.second;
    dia = datetime.day;
    mes = datetime.month;
    ano = datetime.year;
    get_date_time = String(dia) + "/" + String(mes) +"/"+ String(ano) +"-"+ String(hh) +":"+ String(mm)+":"+ String(ss) + "\r\n";
    return get_date_time;
}

void printUint16Hex(uint16_t value) {
    Serial.print(value < 4096 ? "0" : "");
    Serial.print(value < 256 ? "0" : "");
    Serial.print(value < 16 ? "0" : "");
    Serial.print(value, HEX);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
    Serial.print("Serial: 0x");
    printUint16Hex(serial0);
    printUint16Hex(serial1);
    printUint16Hex(serial2);
    Serial.println();
}

// Initialize SD card
void initSDCard(){
    
    Serial.println("Initializing Micro SD card...");
    
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
    
    timelog = get_date();            

  myFile = SD.open("data.txt", FILE_WRITE);
    // if the file opened okay, write to it:
  if (myFile) {
    
    Serial.println("Writing to data.txt...");
    
    myFile.print("Data collection begin. Date: ");
    
    myFile.println(timelog.c_str());
    myFile.println("Millis; Systolic; Diastolic; Heart Rate; SPO2; CO2");
	myFile.flush();
    // close the file:
    myFile.close();
    
    Serial.println("done.");
    
  } else {
    // if the file didn't open, print an error:
    
    Serial.println("error opening data.txt");
    
  }
  
}
//Open file stream
// Initialize SD card
void startlog(){
    
  Serial.println("Opening Micro SD card...");
  myFile = SD.open("data.txt", FILE_WRITE);
  
  
}


//deinitialyze SDCARD
void deinitSDCard(){
    
    Serial.println("Closing Micro SD card...");
    
    myFile.flush();
    myFile.close();
	Serial.println("done closing file.");
}

void mfioInterruptHndlr(){
  //Serial.println("i");
}

void enableInterruptPin(){

  //pinMode(mfioPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MAX32664.mfioPin), mfioInterruptHndlr, FALLING);

}

void loadAlgomodeParameters(){

  algomodeInitialiser algoParameters;
  /*  Replace the predefined values with the calibration values taken with a reference spo2 device in a controlled environt.
      Please have a look here for more information, https://pdfserv.maximintegrated.com/en/an/an6921-measuring-blood-pressure-MAX32664D.pdf
      https://github.com/Protocentral/protocentral-pulse-express/blob/master/docs/SpO2-Measurement-Maxim-MAX32664-Sensor-Hub.pdf
  */

  algoParameters.calibValSys[0] = 120;
  algoParameters.calibValSys[1] = 122;
  algoParameters.calibValSys[2] = 125;

  algoParameters.calibValDia[0] = 80;
  algoParameters.calibValDia[1] = 81;
  algoParameters.calibValDia[2] = 82;

  algoParameters.spo2CalibCoefA = 1.5958422;
  algoParameters.spo2CalibCoefB = -34.659664;
  algoParameters.spo2CalibCoefC = 112.68987;

  MAX32664.loadAlgorithmParameters(&algoParameters);
}

void startnewcalib(void)
{       // Initialize sensor
	loadAlgomodeParameters();
	int result = MAX32664.hubBegin();
		
    //Mantenha o dedo pressionado
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN);
    tft.drawString("Insira o Dedo", 2, 30,4);
    delay(1000);
    bool ret = MAX32664.startBPTcalibration(); //max32664 calibration
    Serial.println("Mantenha o dedo - Calib");

    while(!ret){
    delay(1000);//10000
    
    Serial.println("failed calib, please restart");
    
    }

  delay(2000);
    
    Serial.println("start in estimation mode");
    
    ret = MAX32664.configAlgoInEstimationMode();
    while(!ret){

    
    Serial.println("failed est mode");
    
    ret = MAX32664.configAlgoInEstimationMode();
    delay(2000);
  }

    
    if (!find_max30105 && !showError) {
        tft.fillScreen(TFT_BLACK);
        tft.drawString("No detected sensor", 20, 30);
        showError = true;
        return;
    }

    if (showError) {
        return;
    }
}



bool setupMAX30105(void)
{
    // Initialize sensor
	int result = CMD_SUCCESS;
		
    if (!result == CMD_SUCCESS) { //Use default I2C port, 400kHz speed
    tft.setTextColor(TFT_GREEN);
    tft.println("MAX32664 was not found");
        return false;
    }
    tft.setTextColor(TFT_GREEN);
    tft.println("MAX32664 - OK");
    tft.setTextColor(TFT_GREEN);
    find_max30105 = true;
    return true;
}


void loopMAX30105(void)
{
    /////inicio loop max32664////
    if ((millis() - lastTimebmp) > timerDelaybmp) { //reading every 100ms
    
	///get MAX32664 reading 
    uint8_t num_samples = MAX32664.readSamples();
	if(num_samples==0) Serial.println("MAX32664 - No samples to read");
      if(num_samples){
		systol = MAX32664.max32664Output.sys;
		diastol = MAX32664.max32664Output.dia;
		heart = MAX32664.max32664Output.hr;
		oxig = MAX32664.max32664Output.spo2;  	
    }
	
	lastTimebmp = millis();
    }
    /////fim loop max32664////


    /////inicio loop scd41////

    /////fim loop scd41////


    /////inicio loop bmp280////

    /////fim loop bmp280////


    /////inicio loop cartaosd////
    if ((millis() - lastTimebmp) > timerDelaysd) { 

    dataMessage = String(sensor.getTime()) + ";" + String(systol) +";"+ String(diastol) +";"+ String(heart) +";"+ String(oxig) + ";" + String(co2) + "\r\n";
    //Append the data to file
    Serial.println(dataMessage);
    
    myFile.print(dataMessage.c_str());
	myFile.flush();		
	lastTimebmp = millis();
    }
    /////fim loop cartaosd////

    /////inicio loop nuvem////
    if ((millis() - lastTimebmp) > timerDelaynuvem) { 

   	
	lastTimebmp = millis();
    }
    /////fim loop nuvem////

    /////inicio loop display////
    
    /////fim loop display////






	
//exibe as informacoes coletadas
    if (targetTime1 < millis()) {
        tft.fillScreen(TFT_BLACK);
        tft.setTextSize(1);
        snprintf(buff, sizeof(buff), "Sistolica= %d mmHg", MAX32664.max32664Output.sys);
        //snprintf(buff, sizeof(buff), "Sistolica= %d mmHg", systol);
        tft.drawString(buff, 0, 0);
        snprintf(buff, sizeof(buff), "Diastolica= %.2d mmHg", MAX32664.max32664Output.dia);
        //snprintf(buff, sizeof(buff), "Diastolica= %.2d mmHg", diastol);
        tft.drawString(buff, 0, 20);
        snprintf(buff, sizeof(buff), "Freq. Card.= %d BPM", MAX32664.max32664Output.hr);
        //snprintf(buff, sizeof(buff), "Freq. Cardiaca= %d BPM", heart);
        tft.drawString(buff, 0, 40);
        snprintf(buff, sizeof(buff), "Oxigenacao= %.2f %", MAX32664.max32664Output.spo2);
        //snprintf(buff, sizeof(buff), "Oxigenacao= %.2f %", oxig);
        tft.drawString(buff, 0, 60);
        
        targetTime1 += 250;
												  

    }	
	   
		 
    //SCD41
        //write sd
    if ((millis() - lastTimebmp) > timerDelaySCD) { //reading every 5s
    
	///get SCD41 reading 
      uint16_t error;
    char errorMessage[256];

   // delay(5000);

    // Read Measurement
    error = scd4x.readMeasurement(co2, temperature, humidity);
    if (error) {
        Serial.print("Error trying to execute readMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else if (co2 == 0) {
        Serial.println("Invalid sample detected, skipping.");
    } else {
        Serial.print("Co2:");
        Serial.print(co2);
        Serial.print("\t");
        Serial.print("Temperature:");
        Serial.print(temperature);
        Serial.print("\t");
        Serial.print("Humidity:");
        Serial.println(humidity);
    }
						  
	lastTimebmp = millis();
    }

}

// void setupMonitor()
// {
//     esp_adc_cal_characteristics_t adc_chars;
//     esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_7, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
//     //Check type of calibration value used to characterize ADC
//     if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
//         Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
//         Serial.printf("\r\n");
//         vref = adc_chars.vref;
//     } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
//         Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
//     } else {
//         Serial.println("Default Vref: 1100mV");
//     }
//     pinMode(CHARGE_PIN, INPUT);
//     attachInterrupt(CHARGE_PIN, [] {
//         charge_indication = true;
//     }, CHANGE);

//     if (digitalRead(CHARGE_PIN) == LOW) {
//         charge_indication = true;
//     }
// }

bool setupRTC(void)
{
    Wire.beginTransmission(PCF8563_SLAVE_ADDRESS);
    if (Wire.endTransmission() != 0) {
        return false;
    }

    rtc.begin(Wire);

    pinMode(RTC_INT_PIN, INPUT_PULLUP);
    attachInterrupt(RTC_INT_PIN, [] {
        rtcIrq = 1;
    }, FALLING);
    //Use compile time
    RTC_Date compiled = RTC_Date(__DATE__, __TIME__);
    rtc.setDateTime(compiled);
    //Check if the RTC clock matches, if not, use compile time   
    //rtc.check();

    RTC_Date datetime = rtc.getDateTime();
    hh = datetime.hour;
    mm = datetime.minute;
    ss = datetime.second;

    return true;
}

void clickHandle(void)
{
    func_select++;
    func_select = func_select % 2; //%3 *davi
    if (func_select == 0) {
        initial = 1;
        targetTime = 0;
        omm = 99;
        // if (digitalRead(CHARGE_PIN) == LOW) {
        //     charge_indication = true;
        // }
        tft.fillScreen(TFT_BLACK);
    }
    showError = false;
}

void setup(void)
{
    
    Serial.begin(115200);
    
    pinMode(SS, OUTPUT);
    if (!SD.begin(26, 14, 13, 27)) {
    Serial.println("initialization failed!");
    return;
  }
    Serial.println("SD initialization done.");
    tft.init();
    tft.setRotation(1);//3
    tft.setSwapBytes(true);
    tft.pushImage(0, 0,160,80, puc);
    delay(3000);
        
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    
    uint16_t error;
    char errorMessage[256];
    scd4x.begin(Wire);

        // stop potentially previously started measurement
    error = scd4x.stopPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    uint16_t serial0;
    uint16_t serial1;
    uint16_t serial2;
    error = scd4x.getSerialNumber(serial0, serial1, serial2);
    if (error) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        printSerialNumber(serial0, serial1, serial2);
    }

    // Start Measurement
    error = scd4x.startPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute startPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    Serial.println("Waiting for first measurement... (5 sec)");


    button.attachClick(clickHandle);
    
    tft.fillScreen(TFT_BLACK);
    tft.setTextFont(2);//2
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_GREEN);

    
    if (!setupMAX30105()) {
        tft.setTextColor(TFT_RED);
        tft.println("Check MAX30105 FAIL");
        tft.setTextColor(TFT_GREEN);
    } else {
        tft.println("Check MAX30105 PASS");
    }

    if (!setupRTC()) {
        tft.setTextColor(TFT_RED);
        tft.println("Check PCF8563 FAIL");
        tft.setTextColor(TFT_GREEN);
    } else {
        tft.println("Check PCF8563 PASS");
    }

    //setupMonitor();
    // tft.print("Correction Vref=");
    // tft.print(vref);
    // tft.println(" mv");

    setCpuFrequencyMhz(80); //crystal mounted = 40MHz, so freq_min is 10 (40/4)

    initSDCard();
    delay(3000);

tft.println("Conectando ao WiFi");
    func_select = 0;
    targetTime = 0;


    /*INFLUXDB*/
      // Setup wifi
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to wifi");
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

// Set write precision to milliseconds. Leave other parameters default.
client.setWriteOptions(WriteOptions().writePrecision(WritePrecision::MS));



  // Add tags
  sensor.addTag("device", DEVICE);
 // sensor.addTag("SSID", WiFi.SSID());

  // Accurate time is necessary for certificate validation and writing in batches
  // For the fastest time sync find NTP servers in your area: https://www.pool.ntp.org/zone/
  // Syncing progress and the time will be printed to Serial.
  timeSync(TZ_INFO, "a.st1.ntp.br", "0.br.pool.ntp.org", "pool.ntp.org");

  // Check server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
    /**********/
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK); // Note: the new fonts do not draw the background colour

}

String getVoltage()
{
    uint16_t v = analogRead(BATT_ADC_PIN);
    float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
    return String(battery_voltage) + "V";
}


void page1()
{

if (pass == 0){
    tft.fillScreen(TFT_BLACK);
    tft.setTextFont(2);
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_GREEN);
    tft.println("Pronto");
    tft.println("Pressione para iniciar");

    Serial.print("Page 1::");
    Serial.println(func_select);
    pass = 1;
}
//    tft.setTextColor(TFT_GREEN);

    // if (charge_indication) {
    //     charge_indication = false;
    //     if (digitalRead(CHARGE_PIN) == LOW) {
    //         tft.pushImage(140, 55, 16, 16, charge);
    //     } else {
    //         tft.fillRect(140, 55, 16, 16, TFT_BLACK);
    //     }
    // }

    // if (targetTime < millis()) {
    //     RTC_Date datetime = rtc.getDateTime();
    //     hh = datetime.hour;
    //     mm = datetime.minute;
    //     ss = datetime.second;
    //     // Serial.printf("hh:%d mm:%d ss:%d\n", hh, mm, ss);
    //     targetTime = millis() + 1000;
    //     if (ss == 0 || initial) {
    //         initial = 0;
    //         tft.setTextColor(TFT_GREEN, TFT_BLACK);
    //         tft.setCursor (8, 60);
    //         tft.print(__DATE__); // This uses the standard ADAFruit small font
    //     }
    //     tft.setTextColor(TFT_BLUE, TFT_BLACK);
    //     tft.drawCentreString(getVoltage(), 120, 65, 1);


    //     // Update digital time
    //     uint8_t xpos = 6;
    //     uint8_t ypos = 0;
    //     if (omm != mm) { // Only redraw every minute to minimise flicker
    //         // Uncomment ONE of the next 2 lines, using the ghost image demonstrates text overlay as time is drawn over it
    //         tft.setTextColor(0x39C4, TFT_BLACK);  // Leave a 7 segment ghost image, comment out next line!
    //         //tft.setTextColor(TFT_BLACK, TFT_BLACK); // Set font colour to black to wipe image
    //         // Font 7 is to show a pseudo 7 segment display.
    //         // Font 7 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 0 : .
    //         tft.drawString("88:88", xpos, ypos, 7); // Overwrite the text to clear it
    //         tft.setTextColor(0xFBE0, TFT_BLACK); // Orange
    //         omm = mm;

    //         if (hh < 10) xpos += tft.drawChar('0', xpos, ypos, 7);
    //         xpos += tft.drawNumber(hh, xpos, ypos, 7);
    //         xcolon = xpos;
    //         xpos += tft.drawChar(':', xpos, ypos, 7);
    //         if (mm < 10) xpos += tft.drawChar('0', xpos, ypos, 7);
    //         tft.drawNumber(mm, xpos, ypos, 7);
    //     }

    //     if (ss % 2) { // Flash the colon
    //         tft.setTextColor(0x39C4, TFT_BLACK);
    //         xpos += tft.drawChar(':', xcolon, ypos, 7);
    //         tft.setTextColor(0xFBE0, TFT_BLACK);
    //     } else {
    //         tft.drawChar(':', xcolon, ypos, 7);
    //     }
    // }
}

void loop()
{
    button.tick();
    switch (func_select) {
    case 0:
        nocal=0;
        //Serial.println("Page 1");
        page1();
        break;
    case 1:
       if(nocal==0) {
            startnewcalib();
            startlog();					   
            nocal=1;
            }
        loopMAX30105();
        break;
    case 2:
        nocal=0;
        deinitSDCard();
        pass = 0;
        break;
    default:
        break;
    }
}

void drawProgressBar(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint8_t percentage, uint16_t frameColor, uint16_t barColor)
{
    if (percentage == 0) {
        tft.fillRoundRect(x0, y0, w, h, 3, TFT_BLACK);
    }
    uint8_t margin = 2;
    uint16_t barHeight = h - 2 * margin;
    uint16_t barWidth = w - 2 * margin;
    tft.drawRoundRect(x0, y0, w, h, 3, frameColor);
    tft.fillRect(x0 + margin, y0 + margin, barWidth * percentage / 100.0, barHeight, barColor);
}

