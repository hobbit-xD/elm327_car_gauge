#include "BLEDevice.h"
#include <TFT_eSPI.h>  // Include the graphics library for drawing on the display
#include <CST816S.h>

// all the images are created in Photopea, exported using image2cpp website and stored in separate header files
#include "ford_boost.h"

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

String imapRequestCommand = "010B\r";  //Intake manifold absolute pressure pid command return value in kPa
String bpRequestCommand = "0133\r";    //Absolute Barometric Pressure return value in kPa

unsigned int coolantTemperature;
unsigned int barometricPressure = DEFAULT_PRESSURE;
unsigned int bp = 0;
unsigned int imap = DEFAULT_PRESSURE;

int intervalloScansione_pressione = 2700000;
unsigned long tempo_trascorso_pressione;

int menu = 0;

float psi_older[20];  // store a few older readings for smoother readout

unsigned int hexToDec(String hexString) {
  Serial.println("hexToDec function");
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
    Serial.println("Received response: " + response);
    if (response.substring(2, 4) == "05") {
      //Calculating coolant Temperature
      coolantTemperature = hexToDec(response.substring(4)) - 40;

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
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remote BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");
  pClient->setMTU(517);  // set client to request maximum MTU from server (default is 23 otherwise)

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our write characteristic");


  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pReadCharacteristic = pRemoteService->getCharacteristic(txUUID);
  if (pReadCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(txUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our read/notify characteristic");


  // Read the value of the characteristic.
  if (pReadCharacteristic->canRead()) {
    Serial.println("The characteristic can read");
    //String value = pReadCharacteristic->readValue();
    //Serial.print("The characteristic value was: ");
    //Serial.println(value.c_str());
  }

  if (pReadCharacteristic->canNotify()) {
    Serial.println("The characteristic can notify, registering callback: ");
    pReadCharacteristic->registerForNotify(notifyCallback);
  }

  connected = true;
  Serial.println("We are now connected to the BLE Server.");
  Serial.println("ELM327 initializing..");

  // Initialize ELM327 after successful connection
  if (!initializeELM327()) {
    Serial.println("Failed to initialize ELM327");
    connected = false;
    return false;
  }

  Serial.println("ELM327 initialized successfully");
  return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      Serial.println("Device found");
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
    Serial.print("Sending command: ");
    Serial.println(cmd);
    pRemoteCharacteristic->writeValue(cmd, strlen(cmd));
    delay(1000);  // Wait for response
                  // Might want to read the response here and check for success
    // No response is given for some reason.
  }

  return true;  // Return true if initialization is successful
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  touch.begin();
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
}

void loop() {
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server and ELM327 is initialized.");
    } else {
      Serial.println("We have failed to connect to the server or initialize ELM327; there is nothing more we will do.");
    }
    doConnect = false;
    delay(500);
  }

  if (connected) {

    //choose display page
    if (touch.available()) {
      menu++;

      if (menu > 1) {
        menu = 0;
      }
    }

    switch (menu) {
      case 0:
        {
          // Sending the command to request intake manifold pressure
          Serial.println("Sending Imap request command: " + imapRequestCommand);
          pRemoteCharacteristic->writeValue(imapRequestCommand.c_str(), imapRequestCommand.length());

          // Sending the command to request barometric pressure
          if (millis() - tempo_trascorso_pressione >= intervalloScansione_pressione) {
            Serial.println("Sending Bp request command: " + bpRequestCommand);
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
          for (int i = 0; i < 9; i++) {
            psi_older[i] = psi_older[i + 1];
          }
          psi_older[9] = boost_psi;

          float psi_combined = 0;
          for (int i = 0; i < 10; i++) {
            psi_combined = psi_combined + psi_older[i];
          }
          psi_combined = psi_combined / 10;  // final pressure value is the average of last 10 readings

          int image_pressure = map(round(psi_combined), -15, 45, 0, 60);
          image_pressure = constrain(image_pressure, 0, 60);  // restrict the images to 0-60 in case the PSI pressure is outside the range of -15..45

          tft.pushImage(0, 0, 240, 240, boost_gauges_allArray[image_pressure]);  // draw the fullscreen boost gauge image
        }
        break;

      case 1:
        {
          // Now wait for the notification to come in the callback function
          tft.fillScreen(TFT_BLACK);
          tft.setCursor(120, 120, 2);
          // Set the font colour to be white with a black background, set text size multiplier to 1
          tft.setTextColor(TFT_WHITE);
          tft.setTextSize(1);
          // We can now plot text on screen using the "print" class
          tft.print("Bp value: ");
          tft.println(bp);
        }
        break;
    }

  } else if (doScan) {
    BLEDevice::getScan()->start(0);
  }

  delay(50);  // Delay for a second before the next loop
}
