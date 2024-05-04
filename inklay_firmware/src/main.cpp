// INKLAY Firmware

// Config: Version
#define CLIENT_VERSION "0.0.16"

// Config: Channel
// #define CHANNEL_BETA
#define CHANNEL_PRODUCTION

// Config: Bluetooth
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Config: Other
#define WIFITIMEOUT 10000
#define HTTPTIMEOUT 10000
#define DEFAULTDEEPSLEEP 60 /* 300 */
#define BATTERYLOWVOLTAGE 3
#define COLORLEDGREEN RgbColor(0, 255, 0)
#define COLORLEDORANGE RgbColor(255, 153, 0)
#define COLORLEDBLUE RgbColor(0, 0, 255)
#define COLORLEDRED RgbColor(255, 0, 0)
#define COLORLEDOFF RgbColor(0, 0, 0)

// Remove Print Statements in Production
#ifdef CHANNEL_BETA
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// Config:
// This file can be exluded in the public repo
#include "config.h"

// Arduino
#include <Arduino.h>

// LED
#include <NeoPixelBus.h>

// NVS
#include "nvs_flash.h"
#include <nvs.h>

// JSON
#include <ArduinoJson.h>

// Bluetooth:
// The ESP Partion Sheme has to be set to Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// Wifi: Wifi
#include <WiFi.h>
#include <WiFiClientSecure.h>

// OTA Update
#include <Update.h>

// EPDiy: Include EPDiy library
#include "epd_driver.h"
#include "epd_highlevel.h"

// EPDiy: Fonts
#include "robotoRegular16.h"

// EPDiy: Battery
#include "driver/adc.h"
#include "esp_adc_cal.h"

// Local images
// #include "inklay.h"
#include "img_download_app.h"
#include "img_battery.h"
#include "img_error.h"
#include "img_loading.h"
#include "img_missing_device.h"
#include "img_pair.h"

#ifdef CHANNEL_BETA
#include "img_qr_dev.h"
#elif defined(CHANNEL_PRODUCTION)
#include "img_qr_prod.h"
#endif

// Version
const char *clientVersion = CLIENT_VERSION;

// Display Type
const char *clientDisplayType = "9in7_16grey";
// const char *clientDisplayType = "13in3_16grey";

// Config
const char *updateServerHost = UPDATESERVERHOST;
const char *updateServerURL = UPDATESERVERURL;
const char *updateServerHostingHost = UPDATEHOSTINGSERVERHOST;
const char *updateServerHostingBinFolder = UPDATESHOSTINGERVERBINFOLDER;

#ifdef CHANNEL_BETA
const char *clientChannel = "beta";
const char *firebaseDatabaseHost = BETAFIREBSEDATABASEHOST;
const char *firebaseStorageHost = BETAFIREBASESTORAGEHOST;
const char *firebaseStorageURL = BETAFIREBASESTORAGEURL;
const char *firebaseCloudFunctionHost = BETACLOUDFUNCTIONSHOST;
#elif defined(CHANNEL_PRODUCTION)
const char *clientChannel = "production";
const char *firebaseDatabaseHost = PRODFIREBSEDATABASEHOST;
const char *firebaseStorageHost = PRODFIREBASESTORAGEHOST;
const char *firebaseStorageURL = PRODFIREBASESTORAGEURL;
const char *firebaseCloudFunctionHost = PRODCLOUDFUNCTIONSHOST;
#endif

// Timer
unsigned long updateMillis;

// EPDiy: Define EPD state object
EpdiyHighlevelState hl;
uint8_t *fb;

// Mac Address
String macAddress = "";

// Startup Mode
enum StartupMode {
    startUpBatteryLow,
    startUpDebug,
    startUpGetApp,
    startUpWelcome,
    startUpWifi,
    startUpBluetooth,
    startUpDisplayOff
};
StartupMode startUpMode;

// LED: Has to be NeoEsp32I2s0800KbpsMethod method or NeoEsp32BitBang800KbpsMethod in order not to conflict with epdiy!
NeoPixelBus<NeoGrbFeature, NeoEsp32I2s0Ws2812xMethod> strip(1, 3);
NeoGamma<NeoGammaTableMethod> colorGamma;

// GPIO Pins
const gpio_num_t pinRSE = GPIO_NUM_13;   // Reset Button
const gpio_num_t pinBLE = GPIO_NUM_39;   // Bluetooth Button
const gpio_num_t pinOFF= GPIO_NUM_36;    // Off Button
const gpio_num_t pinLED = GPIO_NUM_14;   // LED Power Pin
const gpio_num_t pinBAT = GPIO_NUM_34;   // Battery ADC pin
const gpio_num_t pinTherm = GPIO_NUM_35; // Thermistor pin (Not in use)

uint64_t wakeUpPins = GPIO_SEL_13 | GPIO_SEL_36 | GPIO_SEL_39;

// Button State
int buttonState = LOW;
int lastButtonState = LOW;
unsigned long buttonPressTime = 0;

// Use internal ADC calibration to get more reasonable results
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html
esp_adc_cal_characteristics_t *adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);

// Device Config from Firebase
bool configData = false;
int deepSleepInSeconds = -1;

// Device Stats to Firebase
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int wifiError = 0;
RTC_DATA_ATTR int wifiErrorRestart = 0;

// Bluetooth
BLEServer *pServer = NULL;
bool isReset = false;
bool deviceConnected = false;
bool oldDeviceConnected = false;
RTC_NOINIT_ATTR int bleState = 0;    // bleState 0 = getWifiNetworks(), bleState 1 = initBLE (see Setup for more details)
RTC_NOINIT_ATTR char SSID_List[200]; // Buffer for the list of wifi networks which will be found by getWifiNetworks()

// Deep Sleep
// (Use the ULL (unsigned long long) modifier for longer Sleep Times)
unsigned long long uS_TO_S_FACTOR = 1000000ULL;
unsigned long long uS_TO_MS_FACTOR = 1000ULL;
enum DeepSleepMode {
    forever,
    config,
    restart
};
DeepSleepMode deepSleepMode;
const int deepSleepInSecondsDefault = DEFAULTDEEPSLEEP;

// Functions
void initEPD();
void initDisplay();
void setButtonsAsInterrupts();
void getClientVersion();
enum StartupMode getStartupMode();
const char *getValueByKeyFromNVS(const char *key, const char *storageNameSpace);
void setKeyValueToNVS(const char *key, const char *val, const char *storageNameSpace);
void setDeepSleep(DeepSleepMode deepSleepMode);
void getWifi();
void getDeviceFirmwareVersion(const char *host, const char *url);
void getDeviceConfig(const char *host);
void initLED();
void setLedColor(RgbColor);
int readADC(int pin);
int readADC(int pin, uint8_t sample_count);
float getBatteryVoltage();
float getBatteryPercentage();
void getDeviceImage(const char *host);
void setDeviceStats(const char *host);
uint16_t read16(WiFiClientSecure &client);
uint32_t read32(WiFiClientSecure &client);
uint32_t skip(WiFiClientSecure &client, int32_t bytes);
void getBluetooth();
bool isWakeUpReasonTimer();
void saveWifiNetworks(BLECharacteristic *pCharacteristic);
String getHeaderValue(String header, String headerName);
void updateFirmware(String filename, String serverVersion);
void setError(String msg);
void renderImage(const char *host, const char *url, int16_t posX, int16_t posY, bool with_color);
void renderBatteryLow();
void renderOff();
void renderDebug();
void renderBluetooth();
void renderGetApp();
void renderUpdate(String filename);
void renderNotAvailable();
void renderDisplayError(String msg);
void clearLoadingDisplay();
void updateEPD();

void IRAM_ATTR resetESP() {

    // This function will restart the ESP

    ESP.restart();
}

void IRAM_ATTR checkButtons() {

    // This function is called when an interrupt on a button occurs.
    // Then the pressed button gets detected by checking the voltage.
    // Then a flag is set to the nvs filesystem
    // This flag will be read after restart from the nvs filesystem to identify the startup mode.

    // Reset
    if(digitalRead(pinRSE) == HIGH) {

        ESP.restart();
    }
    
    // Pairing and Debug Mode
    buttonState = digitalRead(pinBLE);

    if (buttonState == HIGH && lastButtonState == LOW) {
        
        // Start Timer
        buttonPressTime = millis();

    } else if (buttonState == LOW && lastButtonState == HIGH) {
        
        unsigned long buttonReleaseTime = millis();
        unsigned long buttonPressDuration = buttonReleaseTime - buttonPressTime;
        
        // Long press detected
        if (buttonPressDuration >= 3000) {
            
            // Set Startup Display Debug Flag
            setKeyValueToNVS("debug", "true", "storage");

            ESP.restart();
        } else {

            // Set Startup Display ble Flag
            setKeyValueToNVS("ble", "true", "storage");

            ESP.restart();
        }
    }
    
    lastButtonState = buttonState;

    // Off
    if(digitalRead(pinOFF) == HIGH) {

        // Set Startup Display Off Flag
        setKeyValueToNVS("off", "true", "storage");

        ESP.restart();
    }
}

void setButtonsAsInterrupts() {

    // This function assigns an Interrupt to a GPIO Pin
    // Interrupts are asynchronus (like events).
    // Interrupts can be assigned to GPIO Pins and called in real-time.
    // Therefore, a reset can be called also during controller setup and not just in the loop of the programm.

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("set button interrups");

    // Define Pin as Input
    pinMode(pinRSE, INPUT_PULLDOWN);
    pinMode(pinBLE, INPUT_PULLDOWN);
    pinMode(pinOFF, INPUT_PULLDOWN);

    // Attach Interupt (reset-button) and assign checkButtons function
    attachInterrupt(pinRSE, checkButtons, CHANGE);
    attachInterrupt(pinBLE, checkButtons, CHANGE);
    attachInterrupt(pinOFF, checkButtons, CHANGE);
}

void getClientVersion() {

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("get client version");

    // Print Version
    DEBUG_PRINT("client version: ");
    DEBUG_PRINTLN(clientVersion);

    // Print Channel
    DEBUG_PRINT("client channel: ");
    DEBUG_PRINTLN(clientChannel);

    // Display Version
    DEBUG_PRINT("display type: ");
    DEBUG_PRINTLN(clientDisplayType);
}

enum StartupMode getStartupMode() {

    // This function returns the startup mode
    // - Low Battery: If voltage is too low
    // - Bluetooth: If BLE button is pressed 
    // - Display Off: If Off button is pressed
    // - Welcome: If no Wi-Fi credentials have been saved yet
    // - Wi-Fi: Everything else (Regular Startup Mode)

    // Each button is configured as interrupt
    // By pressing a button during runtime, the esp gets restarted
    // After restarting, the pressed button can be detected by a flag which is written to the NVS filesystem just before the restart.

    // Each button is also configured as wakeup source for deep sleep.
    // By pressing a button in deep sleep, the esp gets restarted.
    // After restarting, the pressed button can be detected by reading its wakeup status.

    DEBUG_PRINTLN("");
    DEBUG_PRINT("get startup mode: ");

    // Battery Low Mode
    if (getBatteryVoltage() <= BATTERYLOWVOLTAGE) {

        DEBUG_PRINTLN("battery low mode");
        
        return startUpBatteryLow;
    }

    // Debug Mode
    if (strcmp(getValueByKeyFromNVS("debug", "storage"), "true") == 0) {
        
        DEBUG_PRINTLN("debug mode");
        
        // Reset Startup Bluetooth Flag
        setKeyValueToNVS("debug", "false", "storage");
        
        return startUpDebug;
    }

    // Reset Bluetooth state if esp has NOT been restartet by deep sleep or software reset
    if ((esp_reset_reason() != ESP_RST_DEEPSLEEP) && (esp_reset_reason() != ESP_RST_SW)) {
        bleState = 0;
    }

    // Bluetooth Mode
    // Interrupt: Check if ble key has true value in nvs
    // Deep sleep: Check if wakeup source (bitmask) of button
    // Bluetooth State: Check bluetooth state
    if (strcmp(getValueByKeyFromNVS("ble", "storage"), "true") == 0 || esp_sleep_get_ext1_wakeup_status() == 549755813888 || bleState == 1) {
        
        DEBUG_PRINTLN("bluetooth mode");
        
        // Reset Startup Bluetooth Flag
        setKeyValueToNVS("ble", "true", "storage");
        
        return startUpBluetooth;
    }

    // Display Off Mode
    if (strcmp(getValueByKeyFromNVS("off", "storage"), "true") == 0 || esp_sleep_get_ext1_wakeup_status() == 68719476736) {
        
        DEBUG_PRINTLN("display off mode");

        // Reset Startup Display Off Flag
        setKeyValueToNVS("off", "false", "storage");

        return startUpDisplayOff;
    }

    // Setup Mode
    if (strcmp(getValueByKeyFromNVS("wifi_ssid", "storage"), "undefined") == 0) {
        
        DEBUG_PRINTLN("setup mode");

        return startUpGetApp;
    }

    // Welcome Mode
    if (strcmp(getValueByKeyFromNVS("startup", "storage"), "true") == 0) {
        
        DEBUG_PRINTLN("welcome mode");

        return startUpWelcome;
    }
     
    // Wifi Mode
    DEBUG_PRINTLN("wifi mode");
    return startUpWifi;
    
}

void initLED() {

    // This function intialies the LED (WS2812)

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("init led");

    pinMode(pinLED, OUTPUT);
    digitalWrite(pinLED, HIGH);

    // Add a small delay to let the led power up
    delay(5);

    strip.Begin();
    strip.Show();
}

void initEPD() {

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("init epdiy");

    // Initialize EPDiy component with default driver options
    epd_init(EPD_LUT_1K);

    // Set EPD rotation (EPD_ROT_LANDSCAPE, EPD_ROT_PORTRAIT, EPD_ROT_INVERTED_LANDSCAPE, EPD_ROT_INVERTED_PORTRAIT)
    // epd_set_rotation(EPD_ROT_LANDSCAPE);

    // Initialize a state object. This allocates two framebuffers and an update buffer for the display in the external PSRAM.
    // Returns: An initialized state object.
    hl = epd_hl_init(EPD_BUILTIN_WAVEFORM);

    // Get a reference to the front framebuffer. Use this to draw on the framebuffer before updating the screen with epd_hl_update_screen().
    fb = epd_hl_get_framebuffer(&hl);
}

class BleWriteCallback : public BLECharacteristicCallbacks {

    // This callback is listening for data which was send with a bluetooth client (iOS/Android App)
    // The data will be stored in the NVS

    void onWrite(BLECharacteristic *pCharacteristic) {

        DEBUG_PRINTLN("");
        DEBUG_PRINTLN("recieved data from bluetooth");

        std::string value = pCharacteristic->getValue();

        if (value.length() > 0) {
            DEBUG_PRINT("value: ");
            for (int i = 0; i < value.length(); i++) {
                DEBUG_PRINT(value[i]);
            }
            DEBUG_PRINTLN();

            // JSON: ArduinoJson
            // JSON: Allocate the JSON document
            // JSON: Use https://arduinojson.org/v6/assistant/ to calculate the capacity
            // SSID Max Length: 32 char
            // Passphrase Max Length: 63 char
            DynamicJsonDocument doc(192);

            // JSON: Deserialize
            deserializeJson(doc, value);

            // Get SSID and Password
            const char *ssid = doc["ssid"];
            const char *passphrase = doc["passphrase"];

            DEBUG_PRINT("wifi ssid: ");
            DEBUG_PRINTLN(ssid);
            DEBUG_PRINT("wifi password: ");
            DEBUG_PRINTLN(passphrase);

            // Save Key and Value to NVS in Namespace
            setKeyValueToNVS("wifi_ssid", ssid, "storage");
            setKeyValueToNVS("wifi_pw", passphrase, "storage");
        }

        // Use resetESP will result in a crash when writeCharacteristicWithResponse in flutter
        // Therefore a flag is set: On the next onDisconnect the ESP will reset.
        isReset = true;
    }

    //  This callback is setting wifi ssid list which is being listened with a bluetooth client (iOS App)
    void onRead(BLECharacteristic *pCharacteristic) {
        DEBUG_PRINTLN("BLE Read Event");
        saveWifiNetworks(pCharacteristic);
    }
};

class BleConnectCallback: public BLEServerCallbacks {

    // This callback is listening for connect/disconnect to the ble device
    // If the user disconnects, ble advertising will be restarted

    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;

        // This flag will be true after write operation and restart the ESP
        if(isReset) {

            // Set Startup Display Debug Flag
            setKeyValueToNVS("startup", "true", "storage");

            resetESP();
        }
    }

};

void checkToRestartBLEAdvertising() {

    if(startUpMode == startUpBluetooth) {

        // disconnected so advertise
        if (!deviceConnected && oldDeviceConnected) {
            // give the bluetooth stack the chance to get things ready
            delay(500);
            // restart advertising
            pServer->startAdvertising();
            DEBUG_PRINTLN("ble disconnected: start advertising");
            oldDeviceConnected = deviceConnected;
        }
        // connected so reset boolean control
        if (deviceConnected && !oldDeviceConnected) {
            // do stuff here on connecting
            DEBUG_PRINTLN("ble reconnected");
            oldDeviceConnected = deviceConnected;
        }
    }
}

void getBluetooth() {

    // This function establishes a Bluetooth Service
    // Use the iOS App nRF Connect to connect and write utf-8 data
    // Example JSON: {"ssid": "wifi","passphrase":"123456"}

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("get bluetooth");

    // Init the Device
    BLEDevice::init("Inklay");

    // Create a Bluetooth Server
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new BleConnectCallback());

    // Create a Service with the UUID
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Set up the Service as Read / Write
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        // BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE_NR);
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

    // Set the Callback (gets called when data is recieved)
    pCharacteristic->setCallbacks(new BleWriteCallback());

    // Start the service
    pService->start();

    // Start advertising (visibility for ble-scanners)
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
}

void saveWifiNetworks(BLECharacteristic *pCharacteristic) {

    // This function scans Wi-Fi networks
    // And sends this list back to the Bluetooth Client

    DEBUG_PRINTLN("JSON DATA ------> ");
    DEBUG_PRINTLN(SSID_List);
    pCharacteristic->setValue(SSID_List);
    DEBUG_PRINTLN();
}

void getWifiNetworks() {

    // This function scans Wi-Fi networks
    // And sends this list (including macaddress) back to the Bluetooth Client
    // The result looks like this
    // [{"macaddress":"24:6F:28:D1:5D:DC"},{"ssid":"Lukas-Wi-Fi"},{"ssid":"MatchX_MX190x_RHYM"},{"ssid":"mkt-45633"},{"ssid":"UPC7BE5CEC"},{"ssid":"UPC Wi-Free"},{"ssid":"UPC9541478_2GEXT"},{"ssid":"Tukan"},{"ssid":"TP-Link_6568"},{"ssid":"TP-LINK_BBF1"}]

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("scan wifi networks");

    char buf[50];
    char macaddress_buf[18];
    int len = 0; /* Complete String */
    memset(buf, 0, sizeof(buf));
    memset(SSID_List, 0, sizeof(SSID_List));

    DEBUG_PRINT("mac address: ");
    DEBUG_PRINTLN(WiFi.macAddress());

    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
    DEBUG_PRINTLN("scan done");


    if (n == 0) {
        DEBUG_PRINTLN("no networks found");
    } else {
        DEBUG_PRINT(n);
        DEBUG_PRINTLN(" networks found");

        len += sprintf(&SSID_List[len], "%s", "["); 

        // Add Mac Address to the buffer
        WiFi.macAddress().toCharArray(macaddress_buf, WiFi.macAddress().length() + 1);
        // Add Mac Adress to the total String
        len += sprintf(&SSID_List[len], "{\"macaddress\":\"%s\"},", macaddress_buf);

        for (int i = 0; i < 5; ++i) {

            // Print SSID and RSSI for each network found
            DEBUG_PRINT(i + 1);
            DEBUG_PRINT(": ");

            // Get SSID
            String SSID = WiFi.SSID(i);

            // Convert SSID to Char
            SSID.toCharArray(buf, SSID.length() + 1);
            len += sprintf(&SSID_List[len], "{\"ssid\":\"%s\"},", buf);

            DEBUG_PRINT(WiFi.SSID(i));
            DEBUG_PRINT(" (");
            DEBUG_PRINT(WiFi.RSSI(i));
            DEBUG_PRINT(")");
            DEBUG_PRINTLN((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
            //  delay(10);
        }
        len += sprintf(&SSID_List[len - 1], "%s", "]");
        SSID_List[len] = 0;
    }
}

void setKeyValueToNVS(const char *key, const char *val, const char *storageNameSpace) {

    // This function will save key value pairs to NVS

    // DEBUG_PRINTLN("");
    // DEBUG_PRINTLN("save key: " + String(key) + " with value: " + String(val) + " to nvs");

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Open NVS Handle with Namespace
    // DEBUG_PRINTLN("opening nvs handle with namespace: " + String(storageNameSpace));
    nvs_handle my_handle;
    err = nvs_open(storageNameSpace, NVS_READWRITE, &my_handle);

    if (err != ESP_OK) {
        // DEBUG_PRINTLN("error opening nvs handle");
    } else {
        // DEBUG_PRINTLN("succesful opening nvs handle");

        // Write to NVS
        // DEBUG_PRINTLN("write to nvs");
        err = nvs_set_str(my_handle, key, val);

        if (err != ESP_OK) {
            // DEBUG_PRINTLN("write failed");
        } else {
            // DEBUG_PRINTLN("write done");
        }

        // Commit to NVS
        // DEBUG_PRINTLN("commit updates in nvs");
        err = nvs_commit(my_handle);

        if (err != ESP_OK) {
            // DEBUG_PRINTLN("commit failed");
        } else {
            // DEBUG_PRINTLN("commit done");
        }

        // Close NVS Handle
        nvs_close(my_handle);
    }
}

const char *getValueByKeyFromNVS(const char *key, const char *storageNameSpace) {

    // This function can get values by a key from the NVS

    // Open NVS Handle with Namespace
    nvs_handle my_handle;
    esp_err_t err = nvs_open(storageNameSpace, NVS_READWRITE, &my_handle);

    if (err != ESP_OK) {
        DEBUG_PRINTLN("error opening nvs handle");
        return "undefined";
    } else {

        // Read from NVS
        size_t required_size;
        err = nvs_get_str(my_handle, key, NULL, &required_size);

        if (err == ESP_ERR_NVS_NOT_FOUND) {
            return "undefined";
        } else {
            char *value = (char *)malloc(required_size);
            nvs_get_str(my_handle, key, value, &required_size);
            return value;
        }

        // Close NVS Handle
        nvs_close(my_handle);
    }
}

void deleteNVS() {

    // This function will erase NVS data

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("erase nvs");

    esp_err_t err = nvs_flash_erase();

    if (err != ESP_OK) {
        DEBUG_PRINTLN("Erased: Failed");
    } else {
        DEBUG_PRINTLN("Erased: Done");
    }
}

void getWifi() {

    // This function will establish a WiFi connection

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("get wifi");

    // Start Wifi
    WiFi.begin(getValueByKeyFromNVS("wifi_ssid", "storage"), getValueByKeyFromNVS("wifi_pw", "storage"));

    // Keep track of when we started our attempt to get a WiFi connection
    unsigned long startAttemptTime = millis();

    DEBUG_PRINT("connecting to wifi ");

    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFITIMEOUT) {
        delay(500);
        DEBUG_PRINT(".");
    }
    DEBUG_PRINTLN("");

    // Make sure that we're actually connected, otherwise go to deep sleep
    if (WiFi.status() != WL_CONNECTED) {
        
        wifiError++;
        wifiErrorRestart++;

        DEBUG_PRINTLN("connection failed: timeout");
        DEBUG_PRINTLN("wifi error: " + String(wifiError));
        DEBUG_PRINTLN("wifi error restart: " + String(wifiErrorRestart));

        if(wifiErrorRestart == 5) {

            // Reset
            wifiErrorRestart = 0;
            
            // Turn display off
            renderOff();
            setDeepSleep(forever);

            return;
        }
        else {
            setError("Could not connect to " + String(getValueByKeyFromNVS("wifi_ssid", "storage")) + ".");
            return;
        }
    }

    // Revert wifi error restart
    wifiErrorRestart = 0;

    DEBUG_PRINTLN("connected to the wifi network");

    // Get Device Mac Address
    DEBUG_PRINT("mac-address: ");
    DEBUG_PRINTLN(WiFi.macAddress());
    macAddress = WiFi.macAddress();
    macAddress.replace(":", "%3A");
}

void getDeviceFirmwareVersion(const char *host, const char *url) {

    // This function checks the client firmware version with the server firmware version in the channel (beta/production)
    // A http connection is used. Therefore no Certificate is needed.
    // The server responds with version and file (1.0.0, inklay_production_1_0_0.bin)
    // If the server version is identical: Do nothing
    // If the sever version is highter: Start OTA update
    // If the sever version is lower: Start OTA Downgrade

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("check version with inklay update server");

    // Wifi Client (HTTP)
    WiFiClient client;

    // Connect to Host
    DEBUG_PRINTLN("connect to server via port 80");

    // HTTPS Connection: Check if Secure Connection is successfull
    if (!client.connect(host, 80)) {
        // Connection Failed
        DEBUG_PRINTLN("connection failed");

        setError("Update Server: Connection failed.");
        return;
    }

    // HTTPS Connection: Request URL with
    DEBUG_PRINT("requesting url: ");
    DEBUG_PRINT(host);
    DEBUG_PRINTLN(url);

    // Assign URL
    client.print("GET ");
    client.print(url);
    client.println(" HTTP/1.1");

    // Send the HTTP headers
    client.print("Host: ");
    client.println(host);
    client.println("User-Agent: ESP32");
    client.println("Connection: close");
    client.println();

    // HTTPS Connection: Client is connected. Read Headers
    // HTTPS Connection: Header is the first response we get from the Server
    // HTTPS Connection: Example: HTTP/1.1 200 OK: Standard response for successful HTTP requests.

    // Check for Server Timeout
    unsigned long timeout = millis();

    while (client.available() == 0) {
        if (millis() - timeout > HTTPTIMEOUT) {

            // Server not available
            DEBUG_PRINTLN("update server is not responding");

            // Stop the client
            client.stop();

            setError("Update Server is not responding.");

            return;
        }
    }

    // Check the HTTP Response

    while (client.available()) {

        // read first line of header
        String line = client.readStringUntil('\n');

        // remove space, to check if the line is end of headers
        line.trim();

        // If the the line is empty, this is end of headers
        if (!line.length()) {
            // Headers ended
            // Continue
            break;
        }

        if (line.startsWith("HTTP/1.1")) {
            if (!(line.indexOf("200") < 0)) {
                DEBUG_PRINTLN("http response: 200 ok");

                // Continue to read response
            } else {
                // Server not available
                DEBUG_PRINTLN("update server is not available");

                // Stop the client
                client.stop();

                setError("Update Server is not available.");

                return;
            }
        }
    }

    // HTTPS Connection: Once server sends all requested data it will disconnect, then once all received data are read, program will exit the while loop.

    // JSON Parse: ArduinoJson
    // JSON Parse: Allocate the JSON document
    // JSON Parse: Use https://arduinojson.org/v6/assistant/ to calculate the capacity
    StaticJsonDocument<192> doc;

    // JSON Parse: Parse JSON object and receive possible errors
    DeserializationError error = deserializeJson(doc, client);

    // Check for Deserialization Errors
    if (error) {
        DEBUG_PRINT(("deserialization failed with error: "));
        DEBUG_PRINTLN(error.c_str());

        setError("Update Server: Deserialization failed with error: " + String(error.c_str()) + ".");

        return;
    } else {
        DEBUG_PRINTLN("serialization successful");
    }

    // JSON Parse: Assign the data
    const char *serverVersion = doc[clientChannel]["version"];
    const char *serverFilename = doc[clientChannel]["file"];

    DEBUG_PRINT("received: server version: ");
    DEBUG_PRINTLN(serverVersion);

    DEBUG_PRINT("received: server file name: ");
    DEBUG_PRINTLN(serverFilename);

    // Check if server version is identical, bigger or lower than the client version
    String X[3] = {"", "", ""};
    String Y[3] = {"", "", ""};

    int m = 0;
    for (int i = 0; serverVersion[i] != '\0'; i++) {
        if (serverVersion[i] == '.') {
            m++;
            continue;
        }
        X[m].concat(serverVersion[i]);
    }

    m = 0;
    for (int i = 0; clientVersion[i] != '\0'; i++) {
        if (clientVersion[i] == '.') {
            m++;
            continue;
        }
        Y[m].concat(clientVersion[i]);
    }

    if ((X[0].toInt() == Y[0].toInt()) && (X[1].toInt() == Y[1].toInt()) && (X[2].toInt() == Y[2].toInt())) {
        // identical
        DEBUG_PRINT("server version identical - keep client version: ");
        DEBUG_PRINTLN(clientVersion);
    }

    else if ((X[0].toInt() > Y[0].toInt()) || (X[0].toInt() == Y[0].toInt() && X[1].toInt() > Y[1].toInt()) || (X[0].toInt() == Y[0].toInt() && X[1].toInt() == Y[1].toInt() && X[2].toInt() > Y[2].toInt())) {
        // Bigger, Start OTA Upgrade
        DEBUG_PRINT("server version higher - start ota update to ");
        DEBUG_PRINT(clientChannel);
        DEBUG_PRINT("/");
        DEBUG_PRINT(clientDisplayType);
        DEBUG_PRINT("/");
        DEBUG_PRINTLN(serverFilename);
        updateFirmware(String(serverFilename), String(serverVersion));
    }

    else if ((X[0].toInt() < Y[0].toInt()) || (X[0].toInt() == Y[0].toInt() && X[1].toInt() < Y[1].toInt()) || (X[0].toInt() == Y[0].toInt() && X[1].toInt() == Y[1].toInt() && X[2].toInt() < Y[2].toInt())) {
        // Lower, Start OTA Downgrade
        DEBUG_PRINT("server version lower - start ota downgrade to ");
        DEBUG_PRINT(clientChannel);
        DEBUG_PRINT("/");
        DEBUG_PRINT(clientDisplayType);
        DEBUG_PRINT("/");
        DEBUG_PRINTLN(serverFilename);
        updateFirmware(String(serverFilename), String(serverVersion));
    }

    // close
    client.stop();

    DEBUG_PRINTLN("");
    DEBUG_PRINT("checkVersion took ");
    DEBUG_PRINT((millis() - updateMillis));
    DEBUG_PRINTLN("ms");
}

void updateFirmware(String filename, String serverVersion) {

    // This function performs an over the air update (HTTP).
    // The binary on the server will be downloaded and installed

    // The url structure to the binary: [host]/[path]/[channel]/[displayType]/[filename]
    // Example XXX

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("exec ota");

    // Activate the LED
    setLedColor(COLORLEDORANGE);

    long contentLength = 0;
    bool isValidContentType = false;

    // Host
    const char *host = updateServerHostingHost;

    // URL to the binary folder
    String path = updateServerHostingBinFolder;

    // Render Debug Page
    renderUpdate(serverVersion);

    // Wifi Client (HTTP)
    WiFiClientSecure client;

    // Add Certificate
    client.setInsecure();

    DEBUG_PRINTLN("connect to server via port 80");

    // Connect the update
    if (client.connect(host, 443)) {

        DEBUG_PRINT("requesting url: ");
        DEBUG_PRINTLN(host + path + clientChannel + "/" + clientDisplayType + "/" + filename);

        // Get the contents of the bin file
        client.print(String("GET ") + path + clientChannel + "/" + clientDisplayType + "/" + filename + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Cache-Control: no-cache\r\n" + "Connection: close\r\n\r\n");

        unsigned long timeout = millis();

        while (client.available() == 0) {
            if (millis() - timeout > HTTPTIMEOUT) {

                // Server not available
                DEBUG_PRINTLN("update server timeout!");

                // Show error page
                setError("Update failed. Server not responding.");

                // Stop the client
                client.stop();
                return;
            }
        }

        while (client.available()) {

            // read headers
            String line = client.readStringUntil('\n');

            // remove space, to check if the line is end of headers
            line.trim();

            // bring header line to lowercase for propper checking (if webserver would change lower, uppercase of headers)
            line.toLowerCase();

            // If the the line is empty, this is end of headers
            // break the while and feed the remaining "client" to the Update.writeStream();
            if (!line.length()) {
                // Headers ended
                // Get the OTA started
                break;
            }

            // Check if the HTTP Response is 200 (OK)
            // else break and Exit Update
            if (line.startsWith("http/1.1")) {
                if (line.indexOf("200") < 0) {
                    DEBUG_PRINTLN(line);
                    DEBUG_PRINTLN("update failed: server not responding");

                    // Show error page
                    setError("Update failed. Server not responding.");

                    break;
                }
            }

            // extract headers: Get content length
            if (line.startsWith("content-length: ")) {
                contentLength = atol((getHeaderValue(line, "content-length: ")).c_str());
                DEBUG_PRINTLN("got " + String(contentLength) + " bytes from server");
            }

            // extract headers: Get content type
            if (line.startsWith("content-type: ")) {
                String contentType = getHeaderValue(line, "content-type: ");
                DEBUG_PRINTLN("got " + contentType + " payload");
                if (contentType == "application/octet-stream") {
                    isValidContentType = true;
                }
            }
        }
    } else {

        // Server not available
        DEBUG_PRINTLN("connection to " + String(host) + " failed.");

        // Show error page
        setError("Update failed. Server not responding.");
    }

    // Check what is the contentLength and if content type is "application/octet-stream"
    DEBUG_PRINTLN("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

    // check contentLength and content type
    if (contentLength && isValidContentType) {
        // Check if there is enough space to OTA update
        bool canBegin = Update.begin(contentLength);

        if (canBegin) {
            DEBUG_PRINTLN("begin update. this may take 2-5 mins to complete. please wait.");

            // No activity would appear on the Serial monitor
            size_t written = Update.writeStream(client);

            if (written == contentLength) {
                DEBUG_PRINTLN("update sucessfull: written : " + String(written));
            } else {
                DEBUG_PRINTLN("update failed: written only : " + String(written) + "/" + String(contentLength));

                // Show error page
                setError("Update failed.");
            }

            if (Update.end()) {

                if (Update.isFinished()) {
                    // Sucessfull Update
                    DEBUG_PRINTLN("update successfull: rebooting");
                    resetESP();
                } else {
                    DEBUG_PRINTLN("update failed: something went wrong");
                }
            } else {
                DEBUG_PRINTLN("update failed: error #: " + String(Update.getError()));
            }
        } else {

            // Not enough space to begin OTA
            DEBUG_PRINTLN("Not enough space to begin OTA");

            // Show error page
            setError("Update failed. Not enough space to begin OTA.");

            client.flush();
        }
    } else {
        DEBUG_PRINTLN("there was no content in the response");

        // Show error page
        setError("Update failed. There was no content in the response.");

        client.flush();
    }
}

void getDeviceConfig(const char *host) {

    // This function loads the device config data from Firebase (Channel, Display Type, Deep Sleep in Seconds, GMT Time Zone Offset in Seconds)
    // If the channel version is different: A version check (checkVersion) will be performed again and later the OTA process will be started.
    // If the display type is different: A version check (checkVersion) will be performed again and later the OTA process will be started.

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("get device config from firebase");

    // Wifi Client for secure connection (HTTPS)
    WiFiClientSecure client;

    // Add Certificate
    client.setInsecure();

    // Connect to Host
    DEBUG_PRINTLN("connect to server via port 443");

    // HTTPS Connection: Check if Secure Connection is successfull
    if (!client.connect(host, 443)) {
        // Connection Failed
        char err_buf[100];

        if (client.lastError(err_buf, 100) < 0) {

            DEBUG_PRINT("connection failed with error: ");
            DEBUG_PRINTLN(err_buf);

            setError("Firebase Database: Connection failed with error: " + String(err_buf) + ".");
        } else {
            DEBUG_PRINTLN("connection failed");

            setError("Firebase Database: Connection failed.");
        }

        return;
    }

    // HTTPS Connection: Request URL with
    String url = "/devices/" + macAddress + "/config.json";
    DEBUG_PRINT("requesting url: ");
    DEBUG_PRINTLN(host + url);

    // Assign URL
    client.println("GET " + url + " HTTP/1.1");

    // Send the HTTP headers
    client.print("Host: ");
    client.println(host);
    client.println("User-Agent: ESP32");
    client.println("Connection: close");
    client.println();

    // HTTPS Connection: Client is connected. Read Headers
    // HTTPS Connection: Header is the first response we get from the Server
    // HTTPS Connection: Example: HTTP/1.1 200 OK: Standard response for successful HTTP requests.

    // Check for Server Timeout
    unsigned long timeout = millis();

    while (client.available() == 0) {
        if (millis() - timeout > HTTPTIMEOUT) {

            // Server not available
            DEBUG_PRINTLN("firebase database is not responding");

            // Stop the client
            client.stop();

            setError("Firebase Database is not responding.");

            return;
        }
    }

    // Check the HTTP Response
    bool httpError = false;

    while (client.available()) {

        // read first line of header
        String line = client.readStringUntil('\n');

        // remove space, to check if the line is end of headers
        line.trim();

        // If the the line is empty, this is end of headers
        if (!line.length()) {
            // Headers ended
            // Continue
            break;
        }

        if (line.startsWith("HTTP/1.1")) {
            if (!(line.indexOf("200") < 0)) {
                DEBUG_PRINTLN("http response: 200 ok");

                httpError = false;

                // Continue to read response
            } else {
                DEBUG_PRINTLN("http response: error");

                httpError = true;

                // Continue to read response
            }
        }
    }

    // HTTPS Connection: Once server sends all requested data it will disconnect, then once all received data are read, program will exit the while loop.

    // JSON Parse: ArduinoJson
    // JSON Parse: Allocate the JSON document
    // JSON Parse: Use https://arduinojson.org/v6/assistant/ to calculate the capacity
    DynamicJsonDocument doc(480);

    // JSON Parse: Parse JSON object and receive possible errors
    DeserializationError error = deserializeJson(doc, client, DeserializationOption::NestingLimit(2));

    // Check for Deserialization Errors
    if (error) {
        DEBUG_PRINT(("deserialization failed with error: "));
        DEBUG_PRINTLN(error.c_str());

        setError("Firebase Database: Deserialization failed with error: " + String(error.c_str()));

        return;
    } else {
        DEBUG_PRINTLN("serialization successful");
    }

    // Check if the JSON Doc is empty
    if (doc.isNull()) {

        DEBUG_PRINTLN("this device has not been setup.");

        // Display not available
        renderNotAvailable();

        // Deep Sleep
        setDeepSleep(forever);

        return;
    }

    // Read error response
    if (httpError) {

        String error = doc["error"];
        DEBUG_PRINTLN(error);

        setError("Firebase Database: " + error);
        return;
    }

    // JSON Parse: Get the data
    const char *serverChannel = doc["channel"];
    const char *configDisplayType = doc["displayType"];
    deepSleepInSeconds = doc["deepSleepInSeconds"];

    // JSON Parse: Print the result
    DEBUG_PRINT("received: channel: ");
    DEBUG_PRINTLN(serverChannel);
    DEBUG_PRINT("received: display type: ");
    DEBUG_PRINTLN(configDisplayType);
    DEBUG_PRINT("received: deep sleep in seconds: ");
    DEBUG_PRINTLN(deepSleepInSeconds);

    configData = true;

    // close
    client.stop();

    // Check Channel
    // Start OTA update, if config channel and client channel are not identical

    DEBUG_PRINTLN("check client channel with server channel");

    if (strcmp(serverChannel, clientChannel) != 0) {

        DEBUG_PRINT("server channel (");
        DEBUG_PRINT(serverChannel);
        DEBUG_PRINT(") is different to client channel (");
        DEBUG_PRINT(clientChannel);
        DEBUG_PRINTLN(")");

        // Force Update
        clientVersion = "0";

        // Overwite the client channel
        clientChannel = serverChannel;

        // Check if display type also has changed
        if (strcmp(configDisplayType, clientDisplayType) != 0) {
            DEBUG_PRINT("server display type (");
            DEBUG_PRINT(configDisplayType);
            DEBUG_PRINT(") is different to client display type (");
            DEBUG_PRINT(clientDisplayType);
            DEBUG_PRINTLN(")");

            // Overwite the client display type
            clientDisplayType = configDisplayType;
        }

        // Check Version again
        getDeviceFirmwareVersion(updateServerHost, updateServerURL);
    }

    // Identical channel
    DEBUG_PRINT("server channel identical - keep client channel: ");
    DEBUG_PRINTLN(clientChannel);

    DEBUG_PRINTLN("check display type with server display type");

    if (strcmp(configDisplayType, clientDisplayType) != 0) {

        DEBUG_PRINT("server display type (");
        DEBUG_PRINT(configDisplayType);
        DEBUG_PRINT(") is different to client display type (");
        DEBUG_PRINT(clientDisplayType);
        DEBUG_PRINTLN(")");

        // Force Update
        clientVersion = "0";

        // Overwite the client display type
        clientDisplayType = configDisplayType;

        // Check Version again
        getDeviceFirmwareVersion(updateServerHost, updateServerURL);
    }

    // Identical display type
    DEBUG_PRINT("server display type identical - keep client display type: ");
    DEBUG_PRINTLN(clientDisplayType);

    DEBUG_PRINTLN("");
    DEBUG_PRINT("getDeviceConfig took ");
    DEBUG_PRINT((millis() - updateMillis));
    DEBUG_PRINTLN("ms");
}

float getBatteryVoltage() {

    int average = readADC(pinBAT);

    // convert the value to resistance
    // 4095 for 12-bits, 2047 for 11-bits, 1023 for 10-bits, 511 for 9 bits.
    float value = (float)esp_adc_cal_raw_to_voltage(average, adc_chars) * 2 / 1000;

    return value;
}

float getBatteryPercentage() {

    // This function returns the battery voltage as percentage.

    // Voltage Max / Voltage Min
    float voltageMax = 4200;
    float voltageMin = 3000;

    // Percentage Range 0 - rangeMax
    float rangeMax = voltageMax - voltageMin;

    // Current Voltage in Range
    float voltageInRange = (getBatteryVoltage() * 1000) - voltageMin;

    // Current Voltage in Percentage
    float voltagePercentage = (100 / rangeMax) * voltageInRange;

    // Crop Percentage
    // Voltage can be larger than 4200. For example if device is connected over usb.
    if (voltagePercentage > 100) {
        voltagePercentage = 100;
    }

    if (voltagePercentage < 0) {
        voltagePercentage = 0;
    }

    // Return battery percentage
    return voltagePercentage;
}

void setDeviceStats(const char *host) {

    // This function sends device stats to Firebase (battery voltage, battery percentage)
    // Note: For sucessful PUT / POST request it is necessary to listen and wait for the response of the Server

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("send device stats");

    // Wifi Client for secure connection (HTTPS)
    WiFiClientSecure client;

    // Add Certificate to client
    client.setInsecure();

    // Connect to Host
    DEBUG_PRINTLN("connect to server via port 443");

    // HTTPS Connection: Check if Secure Connection is successfull
    if (!client.connect(host, 443)) {
        // Connection Failed
        char err_buf[100];

        if (client.lastError(err_buf, 100) < 0) {

            DEBUG_PRINT("connection failed with error: ");
            DEBUG_PRINTLN(err_buf);

            setError("Firebase Database: Connection failed with error: " + String(err_buf) + ".");
        } else {
            DEBUG_PRINTLN("connection failed");

            setError("Firebase Database: Connection failed.");
        }

        return;
    }

    // JSON Parse: ArduinoJson
    // JSON Parse: Allocate the JSON document
    // JSON Parse: Use https://arduinojson.org/v6/assistant/ to calculate the capacity
    DynamicJsonDocument doc(348);

    float batteryPercentage = getBatteryPercentage();
    float batteryVoltage = getBatteryVoltage();

    // Asign the data
    doc["channel"] = clientChannel;
    doc["version"] = clientVersion;
    doc["displayType"] = clientDisplayType;
    doc["batteryPercentage"] = batteryPercentage;
    doc["batteryVoltage"] = batteryVoltage;
    doc["wifiError"] = wifiError;
    doc["bootCount"] = String(bootCount);

    // HTTPS Connection: Request URL with
    String url = "/devices/" + macAddress + "/stats.json";
    DEBUG_PRINT("requesting url: ");
    DEBUG_PRINTLN(host + url);

    DEBUG_PRINT("sent: client channel: ");
    DEBUG_PRINTLN(clientChannel);
    DEBUG_PRINT("sent: client version: ");
    DEBUG_PRINTLN(clientVersion);
    DEBUG_PRINT("sent: client display type: ");
    DEBUG_PRINTLN(clientDisplayType);
    DEBUG_PRINT("sent: battery percentage: ");
    DEBUG_PRINTLN(batteryPercentage);
    DEBUG_PRINT("sent: battery voltage: ");
    DEBUG_PRINTLN(batteryVoltage);
    DEBUG_PRINT("sent: boot count: ");
    DEBUG_PRINTLN(bootCount);
    DEBUG_PRINT("sent: wifi error: ");
    DEBUG_PRINTLN(wifiError);

    // Assign URL
    client.println("PUT " + url + " HTTP/1.1");

    // Send the HTTP headers
    client.print("Host: ");
    client.println(host);
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(measureJson(doc));
    client.println("Content-Type: application/json");
    client.println();

    // Send JSON document in body
    serializeJson(doc, client);

    // Wait for response
    while (client.connected()) {
        String line = client.readStringUntil('\n');
        if (line == "\r") {
            DEBUG_PRINTLN("headers received");
            break;
        }
    }

    // Read response from Server
    // It seems that the server cancels the requests if the client prematurely closes the connection.
    // This explains why you need to wait for the complete response before calling client.stop().
    while (client.available()) {
        char c = client.read();
        // Serial.write(c);
    }
    // DEBUG_PRINTLN("");

    client.stop();
}

void getDeviceImage(const char *host) {

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("get image from firebase");

    // Setup Wifi Client for secure connection (HTTPS)
    WiFiClientSecure client;

    // Set client insecure
    client.setInsecure();

    // Connect to Host
    DEBUG_PRINTLN("connect to server via port 80");

    // HTTPS Connection
    if (!client.connect(host, 443)) {

        // Connection Failed
        DEBUG_PRINTLN("connection failed");

        setError("Update Server: Connection failed.");
        return;
    }

    // JSON Parse: ArduinoJson
    // JSON Parse: Allocate the JSON request document
    // JSON Parse: Use https://arduinojson.org/v6/assistant/ to calculate the capacity
    DynamicJsonDocument docReq(48);

    // Get macAddress and convert to char
    char bufferMacAddress[18];
    WiFi.macAddress().toCharArray(bufferMacAddress, sizeof(bufferMacAddress));

    // Add macAddress to json doc
    docReq["macAddress"] = bufferMacAddress;

    // HTTPS Connection: Request URL with
    String url = "/getImage";
    DEBUG_PRINT("requesting url: ");
    DEBUG_PRINTLN(host + url);

    DEBUG_PRINT("sent macaddress: ");
    DEBUG_PRINTLN(bufferMacAddress);

    // Assign URL
    client.println("POST " + url + " HTTP/1.1");

    // Send the HTTP headers
    client.print("Host: ");
    client.println(host);
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(measureJson(docReq));
    client.println("Content-Type: application/json");
    client.println();

    // Send JSON document as json body
    serializeJson(docReq, client);

    // HTTPS Connection: Client is connected. Read Headers
    // HTTPS Connection: Header is the first response we get from the Server
    // HTTPS Connection: Example: HTTP/1.1 200 OK: Standard response for successful HTTP requests.

    // Check for Server Timeout
    unsigned long timeout = millis();

    while (client.available() == 0) {
        if (millis() - timeout > HTTPTIMEOUT) {

            // Server not available
            DEBUG_PRINTLN("firebase database is not responding");

            // Stop the client
            client.stop();

            setError("Firebase Database is not responding.");

            return;
        }
    }

    // Check the HTTP Response
    bool httpError = false;

    while (client.available()) {

        // read first line of header
        String line = client.readStringUntil('\n');

        // remove space, to check if the line is end of headers
        line.trim();

        // If the the line is empty, this is end of headers
        if (!line.length()) {
            // Headers ended
            // Continue
            break;
        }

        if (line.startsWith("HTTP/1.1")) {
            if (!(line.indexOf("200") < 0)) {

                DEBUG_PRINTLN("http response: 200 ok");

                httpError = false;

                // Continue to read response
            } else {

                DEBUG_PRINTLN("http response: error");

                httpError = true;

                // Continue to read response
            }
        }
    }

    // HTTPS Connection: Once server sends all requested data it will disconnect, then once all received data are read, program will exit the while loop.

    // JSON Parse: ArduinoJson
    // JSON Parse: Allocate the JSON responsedocument
    // JSON Parse: Use https://arduinojson.org/v6/assistant/ to calculate the capacity
    DynamicJsonDocument docRes(96);

    // JSON Parse: Parse JSON object and receive possible errors
    DeserializationError error = deserializeJson(docRes, client, DeserializationOption::NestingLimit(2));

    // Check for Deserialization Errors
    if (error) {

        DEBUG_PRINT(("deserialization failed with error: "));
        DEBUG_PRINTLN(error.c_str());

        setError("Firebase Database: Deserialization failed with error: " + String(error.c_str()));

        return;

    } else {
        DEBUG_PRINTLN("serialization successful");
    }

    // Read error response
    if (httpError) {

        String error = docRes["error"];
        DEBUG_PRINTLN(error);

        setError("Firebase Database: " + error);

        return;
    }

    // Read response
    const char *message = docRes["message"];

    // JSON Parse: Print the result
    DEBUG_PRINT("received: message: ");
    DEBUG_PRINTLN(message);

    // close
    client.stop();

    DEBUG_PRINTLN("");
    DEBUG_PRINT("generate image on server took ");
    DEBUG_PRINT((millis() - updateMillis));
    DEBUG_PRINTLN("ms");
}

void setDeepSleep(DeepSleepMode deepSleepMode) {

    // This function puts the device into deep-sleep with low power consumption

    DEBUG_PRINTLN("");
    DEBUG_PRINT("complete update took ");
    DEBUG_PRINT((millis() - updateMillis));
    DEBUG_PRINTLN("ms");

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("set deep sleep");

    // Disconnect wifi before deep-sleep (prevents connecting errors)
    WiFi.disconnect();

    // Defince pin as wakeup source
    // Only RTC IO can be used as a source for external wake source (pins: 0, 2, 4, 12-15, 25-27, 32-39)
    // esp_sleep_enable_ext0_wakeup(pinRST, 1); //1 = High, 0 = Low

    // The first argument of the function is a bitmask of the GPIOs you’ll use as a wake up source, and the second argument defines the logic to wake up the ESP32.
    esp_sleep_enable_ext1_wakeup(wakeUpPins, ESP_EXT1_WAKEUP_ANY_HIGH); // 1 = High, 0 = Low

    switch (deepSleepMode) {
    case forever:

        DEBUG_PRINTLN("deep sleep mode: forever");

        // Do not provide wake up source: esp will sleep forever
        DEBUG_PRINTLN("start deep sleep forerver");

        break;
    case config:

        DEBUG_PRINTLN("deep sleep mode: config");

        // Check if the value could be loaded from Firebase
        if (deepSleepInSeconds < 0) {
            // Set the Timer to a default value of 10 Seconds
            deepSleepInSeconds = deepSleepInSecondsDefault;
        }

        // Set wake up source: calculate deep sleep in micro seconds
        esp_sleep_enable_timer_wakeup(((deepSleepInSeconds*1000)-(millis())) * uS_TO_MS_FACTOR);
        DEBUG_PRINTLN("start deep sleep for " + String(((deepSleepInSeconds*1000)-(millis()))) + " miliseconds");

        break;

    case restart:

        DEBUG_PRINTLN("deep sleep mode: restart");

        // Set the Timer to a default value of 10 Seconds
        deepSleepInSeconds = deepSleepInSecondsDefault;

        // Set wake up source: calculate deep sleep in micro seconds
        esp_sleep_enable_timer_wakeup(60 * uS_TO_S_FACTOR);
        DEBUG_PRINTLN("start deep sleep for " + String(deepSleepInSeconds) + " seconds");

        break;

    default:
        break;
    }

    // Start Deep Sleep
    Serial.flush();
    esp_deep_sleep_start();
}

void setError(String msg) {

    // Print Error Message
    renderDisplayError(msg);

    // Put the device into temporary Deep-Sleep
    setDeepSleep(restart);
}

// Display Rendering

void renderImage(const char *host, const char *url) {

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("load bitmap from firebase storage");

    // Client Connection Flag
    bool connection_ok = false;

    // Setup Wifi Client for secure connection (HTTPS)
    WiFiClientSecure client;

    // Set client insecure
    client.setInsecure();

    // Connect to host
    DEBUG_PRINTLN("connect to server via port 443");

    // Check if connection is successfull
    if (!client.connect(host, 443)) {

        // Connection Failed
        char err_buf[100];

        if (client.lastError(err_buf, 100) < 0) {

            DEBUG_PRINT("connection failed with error: ");
            DEBUG_PRINTLN(err_buf);
        } else {

            DEBUG_PRINTLN("connection failed");
        }

        return;
    }

    // Successfull Connection
    DEBUG_PRINT("requesting url: ");
    DEBUG_PRINTLN("https://" + String(host) + url + "/o/devices%2F" + macAddress + "%2Fimage.bmp?alt=media");

    // Send HTTP GET Request
    client.print(String("GET ") + "https://" + host + url + "/o/devices%2F" + macAddress + "%2Fimage.bmp?alt=media" + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "User-Agent: ESP32\r\n" + "Connection: close\r\n\r\n");

    while (client.connected()) {

        // Read header from the stream line per line
        // The header contains info about Content-Type (image/bmp), Content-Length (in bytes) and more.
        String line = client.readStringUntil('\n');

        if (!connection_ok) {

            // Set connection_ok flag if response is 200 OK
            connection_ok = line.startsWith("HTTP/1.1 200 OK");
        }

        // Stop reading headers
        if (line == "\r") {
            DEBUG_PRINTLN("headers received");
            break;
        }
    }

    // Stop execution when connection error
    if (!connection_ok) {
        DEBUG_PRINTLN("connection error");
        return;
    }

    uint32_t bytes_read;

    // 2 Byte: Bitmap Signatrue 4D42
    read16(client);
    bytes_read += 2;

    // Bitmap File Header

    // 4 Bytes: BMP Header: Filesize
    uint32_t fileSize = read32(client);
    bytes_read += 4;
    DEBUG_PRINT("file size: ");
    DEBUG_PRINT(fileSize);
    DEBUG_PRINTLN(" bytes");

    // 4 Bytes: BMP Header: Reserved: actual value depends on the application that creates the image, if created manually can be 0
    uint32_t creatorBytes = read32(client);
    bytes_read += 4;

    // 4 Bytes: BMP Header: Offset where the pixel array (bitmap data) can be found
    uint32_t imageOffset = read32(client);
    bytes_read += 4;
    DEBUG_PRINT("image data start at byte: ");
    DEBUG_PRINTLN(imageOffset);

    // Bitmap Information Header

    // 4 Bytes: DIB Header: Number of bytes in the DIB header (from this point)
    uint32_t headerSize = read32(client);
    bytes_read += 4;
    DEBUG_PRINT("dib header size: ");
    DEBUG_PRINT(headerSize);
    DEBUG_PRINTLN(" bytes");

    // 4 Bytes: DIB Header: Width of the bitmap in pixels
    uint32_t width = read32(client);
    bytes_read += 4;
    DEBUG_PRINT("width: ");
    DEBUG_PRINT(width);
    DEBUG_PRINTLN("px");

    // 4 Bytes: DIB Header: Height of the bitmap in pixels. Positive for bottom to top pixel order.
    uint32_t height = read32(client);
    bytes_read += 4;
    DEBUG_PRINT("height: ");
    DEBUG_PRINT(height);
    DEBUG_PRINTLN("px");

    // 2 Bytes: DIB Header: Number of color planes being used
    uint16_t planes = read16(client);
    bytes_read += 2;
    DEBUG_PRINT("planes: ");
    DEBUG_PRINTLN(planes);

    // 2 Bytes: DIB Header: The number of bits per pixel, which is the color depth of the image. Typical values are 1, 4, 8, 16, 24 and 32.
    // 8 bit per Pixel = 255 values per Pixel
    uint16_t depth = read16(client);
    bytes_read += 2;
    DEBUG_PRINT("depth: ");
    DEBUG_PRINT(depth);
    DEBUG_PRINTLN(" bit");

    // 4 Bytes: DIB Header: Compression method, Most common is 0 (BI_RGB)
    uint32_t compressionMethod = read32(client);
    bytes_read += 4;
    DEBUG_PRINT("compression method: ");
    DEBUG_PRINTLN(compressionMethod);

    // Bytes read so far
    DEBUG_PRINT("Bytes read so far: ");
    DEBUG_PRINT(bytes_read);
    DEBUG_PRINTLN(" bytes");

    // BMP rows are padded (if needed) to 4-byte boundary
    uint32_t rowSize = (width * depth / 8 + 3) & ~3;
    DEBUG_PRINT("Row size: ");
    DEBUG_PRINT(rowSize);
    DEBUG_PRINTLN(" bytes per row");

    // Skip additional DIB Header Data (Print resolution of the image, Number of colors in the palette)
    bytes_read += skip(client, imageOffset - (4 << depth) - bytes_read); // 54 for regular, diff for colorsimportant
    DEBUG_PRINT("Bytes read so far: ");
    DEBUG_PRINT(bytes_read);
    DEBUG_PRINTLN(" bytes");

    // Loop: Läuft von 0 - 255
    // You can use this function to create empty buffers
    // for (uint16_t pn = 0; pn < (1 << depth); pn++) {
    //   DEBUG_PRINT("Loop");
    //   DEBUG_PRINTLN(pn);
    // }

    uint32_t rowPosition = imageOffset;
    DEBUG_PRINT("Row position: ");
    DEBUG_PRINTLN(rowPosition);

    // Skip additional DIB Header Data (Print resolution of the image, Number of colors in the palette)
    bytes_read += skip(client, rowPosition - bytes_read);
    DEBUG_PRINT("Bytes read so far: ");
    DEBUG_PRINT(bytes_read);
    DEBUG_PRINTLN(" bytes");

    // Clear Loading Screen
    clearLoadingDisplay();

    // Start drawing horizontal rows
    for (uint16_t row = 0; row < height; row++) {

        // Start drawing a pixel line
        for (uint16_t col = 0; col < width; col++) {

            // Get bytes
            uint32_t in_bytes = client.read();

            // If byte is empty, try again in next
            if (in_bytes == -1) {
                col -= 1;
            }

            // DEBUG_PRINT("x:");
            // DEBUG_PRINT(col);
            // DEBUG_PRINT("px y:");
            // DEBUG_PRINT(height-row);
            // DEBUG_PRINT("px color: ");
            // DEBUG_PRINTLN(in_bytes, HEX);

            epd_draw_pixel(col, (height - 1) - row, in_bytes, fb);
        }
    }

    updateEPD();

    DEBUG_PRINTLN("");
    DEBUG_PRINT("image update took ");
    DEBUG_PRINT((millis() - updateMillis));
    DEBUG_PRINTLN("ms");
}

void renderDebug() {

    // Handling chars
    // With snprintf a string can be assembled and stored in a buffer.
    // The placeholders (%) must correspond to the data type
    // https://kuepper.userweb.mwn.de/informatik/printf.pdf

    EpdFontProperties font_props = epd_font_properties_default();
    font_props.flags = EPD_DRAW_ALIGN_LEFT;
    font_props.fg_color = 0;

    int x = 20;
    int y = 50;

    epd_write_string(&robotoRegular16, "Debug Mode", &x, &y, fb, &font_props);

    // Channel
    x = 20;
    y = 150;

    char channel_buffer[250];
    snprintf(channel_buffer, sizeof(channel_buffer), "Channel: %s", clientChannel);
    epd_write_string(&robotoRegular16, channel_buffer, &x, &y, fb, &font_props);

    // Display Type
    x = 20;
    y = 200;

    char displayTypeBuffer[250];
    snprintf(displayTypeBuffer, sizeof(displayTypeBuffer), "Display Type: %s", clientDisplayType);
    epd_write_string(&robotoRegular16, displayTypeBuffer, &x, &y, fb, &font_props);

    // Firmware Version
    x = 20;
    y = 250;

    char version_buffer[250];
    snprintf(version_buffer, sizeof(version_buffer), "Firmware Version: %s", clientVersion);
    epd_write_string(&robotoRegular16, version_buffer, &x, &y, fb, &font_props);

    // Display ID Version
    x = 20;
    y = 300;

    char bufferMacAddress[256];
    WiFi.macAddress().toCharArray(bufferMacAddress, sizeof(bufferMacAddress));

    char idBuffer[250];
    snprintf(idBuffer, sizeof(idBuffer), "Display ID: %s", bufferMacAddress);
    epd_write_string(&robotoRegular16, idBuffer, &x, &y, fb, &font_props);

    // Wi-Fi
    x = 20;
    y = 350;

    char wifiBuffer[250];
    snprintf(wifiBuffer, sizeof(wifiBuffer), "Wi-Fi: %s", getValueByKeyFromNVS("wifi_ssid", "storage"));
    epd_write_string(&robotoRegular16, wifiBuffer, &x, &y, fb, &font_props);

    // Batterie
    x = 20;
    y = 400;

    char batteryBuffer[250];
    snprintf(batteryBuffer, sizeof(batteryBuffer), "Battery: %f", getBatteryVoltage());
    epd_write_string(&robotoRegular16, batteryBuffer, &x, &y, fb, &font_props);

    updateEPD();
}

void renderBluetooth() {

    // Handling chars
    // With snprintf a string can be assembled and stored in a buffer.
    // The placeholders (%) must correspond to the data type
    // https://kuepper.userweb.mwn.de/informatik/printf.pdf

    EpdFontProperties font_props = epd_font_properties_default();
    font_props.flags = EPD_DRAW_ALIGN_LEFT;
    font_props.fg_color = 0;

    int x = 50;
    int y = 775;

    epd_write_string(&robotoRegular16, "Pairing Mode", &x, &y, fb, &font_props);

    // Wi-Fi
    // x = 20;
    // y = 150;

    // char wifiBuffer[250];
    // snprintf(wifiBuffer, sizeof(wifiBuffer), "Wi-Fi: %s", getValueByKeyFromNVS("wifi_ssid", "storage"));
    // epd_write_string(&robotoRegular16, wifiBuffer, &x, &y, fb, &font_props);

    // Draw Image
    EpdRect imgPair = {
        .x = (EPD_WIDTH/2) - (imgPair_width/2),
        .y = (EPD_HEIGHT/2) - (imgPair_height/2),
        .width = imgPair_width,
        .height = imgPair_height};

    epd_copy_to_framebuffer(imgPair, imgPair_data, fb);

    updateEPD();
}

void renderGetApp() {

    // Clear Loading Screen
    clearLoadingDisplay();

    EpdFontProperties font_props = epd_font_properties_default();
    font_props.flags = EPD_DRAW_ALIGN_LEFT;
    font_props.fg_color = 0;

    int x = 219;
    int y = 726;

    epd_write_string(&robotoRegular16, "Download the Inklay App and get started!", &x, &y, fb, &font_props);

    // Draw Image
    EpdRect imgDownloadApp = {
        .x = (EPD_WIDTH/2) - (imgDownloadApp_width/2),
        .y = (EPD_HEIGHT/2) - (imgDownloadApp_height/2),
        .width = imgDownloadApp_width,
        .height = imgDownloadApp_height};

    epd_copy_to_framebuffer(imgDownloadApp, imgDownloadApp_data, fb);

    #ifdef CHANNEL_BETA

        // Draw Image
        EpdRect imgQrDev = {
        .x = 50,
        .y = EPD_HEIGHT - imgQrDev_height - 50,
        .width = imgQrDev_width,
        .height = imgQrDev_height};

        epd_copy_to_framebuffer(imgQrDev, imgQrDev_data, fb);

    #elif defined(CHANNEL_PRODUCTION)

        // Draw Image
        EpdRect imgQrProd = {
        .x = 50,
        .y = EPD_HEIGHT - imgQrProd_height - 50,
        .width = imgQrProd_width,
        .height = imgQrProd_height};

        epd_copy_to_framebuffer(imgQrProd, imgQrProd_data, fb);

    #endif

    updateEPD();
}

void renderNotAvailable() {

    // Clear Loading Screen
    clearLoadingDisplay();

    EpdFontProperties font_props = epd_font_properties_default();
    font_props.flags = EPD_DRAW_ALIGN_LEFT;
    font_props.fg_color = 0;

    int x = 219;
    int y = 705;

    epd_write_string(&robotoRegular16, "Add this Inklay to your", &x, &y, fb, &font_props);

    x = 219;
    y = 747;

    epd_write_string(&robotoRegular16, "account with the Inklay App.", &x, &y, fb, &font_props);

    // x = 20;
    // y = 200;

    // char bufferMacAddress[256];
    // WiFi.macAddress().toCharArray(bufferMacAddress, sizeof(bufferMacAddress));

    // char idBuffer[250];
    // snprintf(idBuffer, sizeof(idBuffer), "Display ID: %s", bufferMacAddress);
    // epd_write_string(&robotoRegular16, idBuffer, &x, &y, fb, &font_props);

    // Draw Image
    EpdRect imgMissingDevice = {
        .x = (EPD_WIDTH/2) - (imgMissingDevice_width/2),
        .y = (EPD_HEIGHT/2) - (imgMissingDevice_height/2),
        .width = imgMissingDevice_width,
        .height = imgMissingDevice_height};

    epd_copy_to_framebuffer(imgMissingDevice, imgMissingDevice_data, fb);

    #ifdef CHANNEL_BETA

        // Draw Image
        EpdRect imgQrDev = {
        .x = 50,
        .y = EPD_HEIGHT - imgQrDev_height - 50,
        .width = imgQrDev_width,
        .height = imgQrDev_height};

        epd_copy_to_framebuffer(imgQrDev, imgQrDev_data, fb);

    #elif defined(CHANNEL_PRODUCTION)

        // Draw Image
        EpdRect imgQrProd = {
        .x = 50,
        .y = EPD_HEIGHT - imgQrProd_height - 50,
        .width = imgQrProd_width,
        .height = imgQrProd_height};

        epd_copy_to_framebuffer(imgQrProd, imgQrProd_data, fb);

    #endif

    updateEPD();
}

void renderBatteryLow() {

    DEBUG_PRINTLN("render display battery low"); 

    int x = 50;
    int y = 775;

    epd_write_default(&robotoRegular16, "Oh no! Battery Low. Please charge.", &x, &y, fb);

    // Draw Image
    EpdRect imgBattery = {
        .x = (EPD_WIDTH/2) - (imgBattery_width/2),
        .y = (EPD_HEIGHT/2) - (imgBattery_height/2),
        .width = imgBattery_width,
        .height = imgBattery_height};

    epd_copy_to_framebuffer(imgBattery, imgBattery_data, fb);

    updateEPD();
}

void renderOff() {

    DEBUG_PRINTLN("render display off");

    int x = 50;
    int y = 775;

    epd_write_default(&robotoRegular16, "Display Off", &x, &y, fb);

    // Draw Image
    EpdRect imgBattery = {
        .x = (EPD_WIDTH/2) - (imgBattery_width/2),
        .y = (EPD_HEIGHT/2) - (imgBattery_height/2),
        .width = imgBattery_width,
        .height = imgBattery_height};

    epd_copy_to_framebuffer(imgBattery, imgBattery_data, fb);

    updateEPD();
}

void renderLoading() {

    DEBUG_PRINTLN("render display loading");

    int x = 50;
    int y = 775;

    epd_write_default(&robotoRegular16, "Loading ...", &x, &y, fb);

    // Draw Image
    EpdRect imgLoading = {
        .x = (EPD_WIDTH/2) - (imgLoading_width/2),
        .y = (EPD_HEIGHT/2) - (imgLoading_height/2),
        .width = imgLoading_width,
        .height = imgLoading_height};

    epd_copy_to_framebuffer(imgLoading, imgLoading_data, fb);

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("update screen");

    // Power on the EPD
    DEBUG_PRINTLN("power on");
    epd_poweron();

    // Clear the whole screen by flashing it.
    epd_clear();

    // Update the EPD screen to match the content of the front frame buffer.
    // And define the draw mode
    epd_hl_update_screen(&hl, MODE_GL16, 25);

    // Power OFF
    DEBUG_PRINTLN("power off");
    epd_poweroff();

}

void renderUpdate(String filename) {

    // Clear Loading Screen
    clearLoadingDisplay();

    EpdFontProperties font_props = epd_font_properties_default();
    font_props.flags = EPD_DRAW_ALIGN_LEFT;
    font_props.fg_color = 0;

    int x = 50;
    int y = 733;

    char bufferFilename[256];
    filename.toCharArray(bufferFilename, sizeof(bufferFilename));

    char updateBuffer[250];
    snprintf(updateBuffer, sizeof(updateBuffer), "Updating Inklay to %s %s", clientChannel, bufferFilename);
    epd_write_string(&robotoRegular16, updateBuffer, &x, &y, fb, &font_props);

    x = 50;
    y = 775;

    epd_write_string(&robotoRegular16, "Please wait a few minutes.", &x, &y, fb, &font_props);

    // Draw Image
    EpdRect imgLoading = {
        .x = (EPD_WIDTH/2) - (imgLoading_width/2),
        .y = (EPD_HEIGHT/2) - (imgLoading_height/2),
        .width = imgLoading_width,
        .height = imgLoading_height};

    epd_copy_to_framebuffer(imgLoading, imgLoading_data, fb);

    updateEPD();
}

void renderDisplayError(String msg) {

    // Clear Loading Screen
    clearLoadingDisplay();

    EpdFontProperties font_props = epd_font_properties_default();
    font_props.flags = EPD_DRAW_ALIGN_LEFT;
    font_props.fg_color = 0;

    int x = 50;
    int y = 733;

    char msgBuffer[256];
    msg.toCharArray(msgBuffer, sizeof(msgBuffer));

    char errorBuffer[250];
    snprintf(errorBuffer, sizeof(errorBuffer), "%s", msgBuffer);
    epd_write_string(&robotoRegular16, errorBuffer, &x, &y, fb, &font_props);

    x = 50;
    y = 775;

    epd_write_string(&robotoRegular16, "Inklay will restart in 1 minute.", &x, &y, fb, &font_props);

    // Draw Image
    EpdRect imgError = {
        .x = (EPD_WIDTH/2) - (imgError_width/2),
        .y = (EPD_HEIGHT/2) - (imgError_height/2),
        .width = imgError_width,
        .height = imgError_height};

    epd_copy_to_framebuffer(imgError, imgError_data, fb);

    updateEPD();
}

void updateEPD() {

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("update screen");

    // Power on the EPD
    DEBUG_PRINTLN("power on");
    epd_poweron();

    // Clear the whole screen by flashing it.
    epd_clear();

    // Update the EPD screen to match the content of the front frame buffer.
    // And define the draw mode
    epd_hl_update_screen(&hl, MODE_GL16, 25);

    // Power OFF
    DEBUG_PRINTLN("power off");
    epd_poweroff();

    // Deinit EPD (Important for Deep Sleep)
    DEBUG_PRINTLN("deinit screen");
    epd_deinit();
}

// Helper Functions

void clearLoadingDisplay() {
    
    // Clear Loading Screen
    // Only clear screen if the user has clicked the reset button and the loading screen was shown.
    if (isWakeUpReasonTimer() == false) {
        epd_hl_set_all_white(&hl);
        epd_hl_update_screen(&hl, MODE_GL16, 25);
    }

}

void setLedColor(RgbColor color) {
    strip.ClearTo(color);
    strip.Show();
}

int readADC(int pin, uint8_t sample_count) {
    uint8_t i;
    float average;
    int samples[sample_count];

    // take N samples in a row, with a slight delay
    for (i = 0; i < sample_count; i++) {
        samples[i] = analogRead(pin);
    }

    // average all the samples out
    average = 0;
    for (i = 0; i < sample_count; i++) {
        average += samples[i];
    }
    average /= sample_count;

    return average;
}

int readADC(int pin) {
    return readADC(pin, 10); // 10 samples by default
}

bool isWakeUpReasonTimer() {

    // This function returns true if the Wakeup Reason was a timer

    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
        // DEBUG_PRINTLN("wakeup caused by external signal using rtc_io");
        return false;
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        // DEBUG_PRINTLN("wakeup caused by external signal using rtc_cntl");
        return false;
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        // DEBUG_PRINTLN("wakeup caused by timer");
        return true;
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        // DEBUG_PRINTLN("wakeup caused by touchpad");
        return false;
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        // DEBUG_PRINTLN("wakeup caused by ulp program");
        return false;
        break;
    default:
        // Serial.printf("wakeup was not caused by deep sleep: %d\n", wakeup_reason);
        return false;
        break;
    }
}

String getHeaderValue(String header, String headerName) {
    return header.substring(strlen(headerName.c_str()));
}

uint32_t skip(WiFiClientSecure &client, int32_t bytes) {
    int32_t remain = bytes;
    uint32_t start = millis();
    while ((client.connected() || client.available()) && (remain > 0)) {
        if (client.available()) {
            int16_t v = client.read();
            remain--;
        } else
            delay(1);
        if (millis() - start > HTTPTIMEOUT)
            break; // don't hang forever
    }
    return bytes - remain;
}

uint16_t read16(WiFiClientSecure &client) {
    // BMP data is stored little-endian, same as Arduino.
    uint16_t result;
    ((uint8_t *)&result)[0] = client.read(); // LSB
    ((uint8_t *)&result)[1] = client.read(); // MSB
    return result;
}

uint32_t read32(WiFiClientSecure &client) {
    // BMP data is stored little-endian, same as Arduino.
    uint32_t result;
    ((uint8_t *)&result)[0] = client.read(); // LSB
    ((uint8_t *)&result)[1] = client.read();
    ((uint8_t *)&result)[2] = client.read();
    ((uint8_t *)&result)[3] = client.read(); // MSB
    return result;
}

// Setup

void setup() {

    // Begin Serial
    Serial.begin(115200);

    // Start timer
    updateMillis = millis();

    // Boot Count
    ++bootCount;

    // Init EPD
    initEPD();

    // Init LED
    initLED();

    // Get display-Version, display-channel and display type
    getClientVersion();

    // Attach Button Interrupts
    setButtonsAsInterrupts();

    switch (getStartupMode()) {
    case startUpBatteryLow:

        // Battery Low Mode
        startUpMode = startUpBatteryLow;

        // Render Battery Low Screen
        renderBatteryLow();

        // Deep Sleep
        setDeepSleep(forever);

        break;
    
    case startUpDebug:

        // Debug Mode
        startUpMode = startUpDebug;

        // LED
        setLedColor(RgbColor(0, 0, 0));
        delay(50);
        setLedColor(COLORLEDRED);

        // Render Battery Low Screen
        renderDebug();

        // Deep Sleep
        setDeepSleep(forever);

        break; 

    case startUpWifi:

        // Wifi Mode
        startUpMode = startUpWifi;

        // Check the Wake-Up Reason from Deep-Slep
        // And only activate LED / render Loading Screen when the User clicked the Reset-Button
        // The LED will not blink when the Controller wakes up from the Timer (to save battery).
        if (isWakeUpReasonTimer() == false) {

            // LED
            setLedColor(COLORLEDGREEN);
            delay(50);
            setLedColor(COLORLEDOFF);

            // Render Loading Screen
            renderLoading();
        }

        // Delete NVS
        // DEBUG_PRINTLN("");
        // DEBUG_PRINTLN("delete values from nvs");
        // deleteNVS();

        // Get a WiFi Connection
        getWifi();

        // Get Version
        getDeviceFirmwareVersion(updateServerHost, updateServerURL);

        // Get Device Config from Firebase
        getDeviceConfig(firebaseDatabaseHost);

        // Get Device Image from Firebase
        getDeviceImage(firebaseCloudFunctionHost);

        // Render Device Image
        renderImage(firebaseStorageHost, firebaseStorageURL);

        // Save Device Stats to Firebase
        setDeviceStats(firebaseDatabaseHost);

        // Deep Sleep
        setDeepSleep(config);

        break;

    case startUpGetApp:

        // Setup Mode
        startUpMode = startUpGetApp;

        // Render Welcome Screen
        renderGetApp();

        // Deep Sleep
        setDeepSleep(forever);

        break;

    case startUpWelcome:

        // Startup Welcome Mode
        startUpMode = startUpWelcome;
        setKeyValueToNVS("startup", "false", "storage");

        // Check the Wake-Up Reason from Deep-Slep
        // And only activate LED / render Loading Screen when the User clicked the Reset-Button
        // The LED will not blink when the Controller wakes up from the Timer (to save battery).
        if (isWakeUpReasonTimer() == false) {

            // LED
            setLedColor(COLORLEDGREEN);
            delay(50);
            setLedColor(COLORLEDOFF);

            // Render Loading Screen
            renderLoading();
        }

        // Get a WiFi Connection
        getWifi();

        // Get Version
        getDeviceFirmwareVersion(updateServerHost, updateServerURL);

        // Get Device Config from Firebase
        getDeviceConfig(firebaseDatabaseHost);

        // Render Device Image
        renderImage(firebaseStorageHost, firebaseStorageURL);

        // Save Device Stats to Firebase
        setDeviceStats(firebaseDatabaseHost);

        // Deep Sleep
        setDeepSleep(config);

        break;

    case startUpDisplayOff:

        // Display Off Mode
        startUpMode = startUpDisplayOff;
        setKeyValueToNVS("off", "false", "storage");

        // LED
        setLedColor(COLORLEDRED);

        // Render Display Off Screen
        renderOff();

        // Delay
        delay(1000);

        // Turn off LED
        setLedColor(RgbColor(0, 0, 0));

        // Deep Sleep
        setDeepSleep(forever);

        break;

    case startUpBluetooth:

        // Bluetooth Mode
        startUpMode = startUpBluetooth;
        setKeyValueToNVS("ble", "false", "storage");

        // LED
        setLedColor(COLORLEDBLUE);

        // Bluetooth and Wifi can not be enabled at the same time as long as -DCONFIG_ARDUINO_ISR_IRAM=1
        // Therefore in bluetooth mode the ESP will work in two states:
        // bleState = 1: Scan the wifi networks and save the networks to SSID_List (which is saved in rtc memory) and do a software reset
        // bleState = 2: The ESP will restart, check the bleState, init Bluetooth and send the SSID_List to the connected bluetooth client (app).

        if (bleState == 0) {

            // Render Bluetooth Screen 
            renderBluetooth();

            // Scan wifi networks
            getWifiNetworks();

            // Set state
            bleState = 1;

            // Reset ESP
            resetESP();

        } else {

            // Set state
            bleState = 0;

            // Start Bluetooth Service to recieve data
            getBluetooth();
        }

        break;

    default:
        break;
    }
}

void loop() {

    // BLE
    checkToRestartBLEAdvertising();
    delay(1000);
}