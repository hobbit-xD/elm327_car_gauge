#include "BLEDevice.h"
#include <TFT_eSPI.h>  // Include the graphics library for drawing on the display
#include <CST816S.h>

// all the images are created in Photopea, exported using image2cpp website and stored in separate header files
#include "ford_boost.h"

#define DEBUG 0

#if DEGUG
#define prt(x) Serial.print(x);
#define prtn(x) Serial.println(x);
#else
#define prt(x)
#define prtn(x)
#endif

#define DEFAULT_PRESSURE 101  // Default atmospheric pressure is 101kPa

// The remote service we wish to connect to.
static BLEUUID serviceUUID("0000fff0-0000-1000-8000-00805f9b34fb");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("0000fff2-0000-1000-8000-00805f9b34fb");  //char write
static BLEUUID txUUID("0000fff1-0000-1000-8000-00805f9b34fb");    // char read notify

TFT_eSPI tft = TFT_eSPI();   // Create object "tft"
CST816S touch(6, 7, 13, 5);  // sda, scl, rst, irq/int


static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLERemoteCharacteristic* pReadCharacteristic;
static BLEAdvertisedDevice* myDevice;

String imapRequestCommand = "010B\r";     //Intake manifold absolute pressure pid command return value in kPa
String bpRequestCommand = "0133\r";       //Absolute Barometric Pressure return value in kPa
String oilTempRequestCommand = "015C\r";  //Engine oil temperature

unsigned int oilTemperature;
unsigned int barometricPressure = DEFAULT_PRESSURE;
unsigned int bp = 0;
unsigned int imap = DEFAULT_PRESSURE;


int intervalloScansione_pressione = 2700000;
unsigned long tempo_trascorso_pressione;

int intervalloScansione_temperatura = 60000;
unsigned long tempo_trascorso_temperatura;

int intervalloScansione = 50;
unsigned long tempo_trascorso_imap;


int menu = 0;

float psi_older[5];  // store a few older readings for smoother readout

unsigned int hexToDec(String hexString) {
  prtn("hexToDec function");
  int ArrayLength = hexString.length() + 1;  //The +1 is for the 0x00h Terminator
  char CharArray[ArrayLength];
  hexString.toCharArray(CharArray, ArrayLength);

  return strtoul(CharArray, NULL, 16);
}
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

  // Process the received data to extract the speed
  if (length > 0) {
    String response = "";

    for (size_t i = 0; i < length; i++) {
      response += (char)pData[i];
    }
    prtn("Received response: " + response);
    if (response.substring(2, 4) == "5C") {
      //Calculating oil Temperature
      oilTemperature = hexToDec(response.substring(4)) - 40;

    } else if (response.substring(2, 4) == "0B") {
      //Calculating Imap
      imap = hexToDec(response.substring(4));

    } else if (response.substring(2, 4) == "33") {
      //Calculating BarometriPressure
      bp = hexToDec(response.substring(4));
    }
  }
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    prtn("onDisconnect");
  }
};

bool connectToServer() {
  prt("Forming a connection to ");
  prtn(myDevice->getAddress().toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  prtn(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remote BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  prtn(" - Connected to server");
  pClient->setMTU(517);  // set client to request maximum MTU from server (default is 23 otherwise)

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    prt("Failed to find our service UUID: ");
    prtn(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  prtn(" - Found our service");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    prt("Failed to find our characteristic UUID: ");
    prtn(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  prtn(" - Found our write characteristic");


  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pReadCharacteristic = pRemoteService->getCharacteristic(txUUID);
  if (pReadCharacteristic == nullptr) {
    prt("Failed to find our characteristic UUID: ");
    prtn(txUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  prtn(" - Found our read/notify characteristic");


  // Read the value of the characteristic.
  if (pReadCharacteristic->canRead()) {
    prtn("The characteristic can read");
    //String value = pReadCharacteristic->readValue();
    //prt("The characteristic value was: ");
    //prtn(value.c_str());
  }

  if (pReadCharacteristic->canNotify()) {
    prtn("The characteristic can notify, registering callback: ");
    pReadCharacteristic->registerForNotify(notifyCallback);
  }

  connected = true;
  prtn("We are now connected to the BLE Server.");
  prtn("ELM327 initializing..");

  // Initialize ELM327 after successful connection
  if (!initializeELM327()) {
    prtn("Failed to initialize ELM327");
    connected = false;
    return false;
  }

  prtn("ELM327 initialized successfully");
  return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    prt("BLE Advertised Device found: ");
    prtn(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      prtn("Device found");
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    }
  }
};

bool initializeELM327() {
  // Commands to initialize ELM327
  const char* initCommands[] = {
    "ATD\r",    // set all to default
    "ATZ\r",    // Reset all
    "ATE0\r",   // Echo off
    "ATL0\r",   // Linefeeds off
    "ATS0\r",   // Spaces off
    "ATH0\r",   // Headers off
    "ATSP0\r",  // Auto select protocol
    "ATDP\r"    // Display current protocol
  };

  for (const char* cmd : initCommands) {
    prt("Sending command: ");
    prtn(cmd);
    pRemoteCharacteristic->writeValue(cmd, strlen(cmd));
    delay(1000);  // Wait for response
                  // Might want to read the response here and check for success
    // No response is given for some reason.
  }

  return true;  // Return true if initialization is successful
}

void setup() {
  Serial.begin(115200);
  prtn("Starting Arduino BLE Client application...");
  touch.begin();
  touch.enable_double_click();   // Enable double-click detection
  tft.init();                    // initialize the display
  tft.setRotation(4);            // set the display rotation, in this rotation, the USB port is on the bottom
  tft.fillScreen(TFT_DARKGREY);  // fill the display with a dark grey color
  tft.setTextFont(3);            // set the font, font number 4 is quite big, font was only used for debugging, final numbers are drawn with images
  tft.setSwapBytes(true);        // Swap the colour byte order when rendering, requires for the used color format

  BLEDevice::init("");

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

  tempo_trascorso_pressione = millis();
  tempo_trascorso_temperatura = millis();
  tempo_trascorso_imap = millis();
}

void loop() {
  if (doConnect == true) {
    if (connectToServer()) {
      prtn("We are now connected to the BLE Server and ELM327 is initialized.");
    } else {
      prtn("We have failed to connect to the server or initialize ELM327; there is nothing more we will do.");
    }
    doConnect = false;
    delay(500);
  }

  if (connected) {

    //choose display page
    if (touch.available()) {
      if (touch.gesture() == "DOUBLE CLICK") {
        menu++;
      }

      if (menu > 1) {
        menu = 0;
      }
    }

    switch (menu) {
      case 0:
        {
          // Sending the command to request intake manifold pressure
          if (millis() - tempo_trascorso_imap >= intervalloScansione) {
            prtn("Sending Imap request command: " + imapRequestCommand);
            pRemoteCharacteristic->writeValue(imapRequestCommand.c_str(), imapRequestCommand.length());
            tempo_trascorso_imap = millis();
          }
          // Sending the command to request barometric pressure
          if ((millis() - tempo_trascorso_pressione >= intervalloScansione_pressione) || bp == 0) {

            prtn("Sending Bp request command: " + bpRequestCommand);
            pRemoteCharacteristic->writeValue(bpRequestCommand.c_str(), bpRequestCommand.length());
            tempo_trascorso_pressione = millis();
          }

          if (bp != 0) {
            barometricPressure = bp;
          }

          int boost_Kpa = imap - barometricPressure;
          //float boost_bar = boost_Kpa / 100.0;
          float boost_psi = boost_Kpa / 6.895;  //boost_bar * 14.504;

          // store a few older measurements
          for (int i = 0; i < 4; i++) {
            psi_older[i] = psi_older[i + 1];
          }
          psi_older[4] = boost_psi;

          float psi_combined = 0;
          for (int i = 0; i < 5; i++) {
            psi_combined = psi_combined + psi_older[i];
          }
          psi_combined = psi_combined / 5;  // final pressure value is the average of last 5 readings

          int image_pressure = map(round(psi_combined), -15, 45, 0, 60);
          image_pressure = constrain(image_pressure, 0, 60);  // restrict the images to 0-60 in case the PSI pressure is outside the range of -15..45

          tft.pushImage(0, 0, 240, 240, boost_gauges_allArray[image_pressure]);  // draw the fullscreen boost gauge image
        }
        break;

      case 1:
        {
          if ((millis() - tempo_trascorso_temperatura >= intervalloScansione_temperatura) || oilTemperature == 0) {
            prtn("Sending Oil temperature request command: " + oilTempRequestCommand);
            pRemoteCharacteristic->writeValue(oilTempRequestCommand.c_str(), oilTempRequestCommand.length());
            tempo_trascorso_temperatura = millis();
          }

          // Now wait for the notification to come in the callback function
          tft.fillScreen(TFT_BLACK);
          tft.setCursor(80, 60, 2);
          // Set the font colour to be white with a black background, set text size multiplier to 1
          tft.setTextColor(TFT_WHITE);
          tft.setTextSize(1);
          // We can now plot text on screen using the "print" class
          tft.print("Bp value: ");
          tft.println(bp);

          tft.setCursor(80, 80, 2);
          tft.print("Oil value: ");
          tft.println(oilTemperature);
        }
        break;
    }

  } else if (doScan) {
    BLEDevice::getScan()->start(0);
  }
}
