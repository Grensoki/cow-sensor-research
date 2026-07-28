//USE THIS FOR LORA TESTING 7/21/26

//allows boards to advertise their data, find other boards, determine which one of the boards has highest battery,
//then sends their individual data to the board with the highest battery

//considerations for power consumptions:
//  make the window for scanning for other boards smaller
//  stop advertising once leader is found (currently boards are advertising for the entire loop) 
//  
#include "LoRaBoards.h"
#include "utilities.h"
#include "tmp117.h"
#include "max30102.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <RadioLib.h>

struct BoardInfo;
struct ReceivedPacket;

#define MAX_NUM_BOARDS 2   //change to number of boards you have for testing
#define SCAN_TIME_MS 2000     //major consideration for power consumption, longer scanning uses more power. CHANGE THIS TO 1000 AFTER TESTING
constexpr uint32_t CYCLE_TIME_MS = 15000;

constexpr uint32_t LEADER_RECEIVE_MS = 4000;
#define THIS_BOARD_ID 1     //change for every board   
#define USE_FAKE_BATTERY 1

//Lora defines
#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ           915.0
#endif

#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER   22
#endif

#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW             125.0
#endif

// Nordic UART-style service UUIDs
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

//TMP117 Globals
TMP117 tempSensor(Wire, 0x48);
bool tmp117Ready = false;
bool tempValid = false;
float myTempF = 0.0f;

//MAX30102 Globals
HRSensor hrSensor(Wire);


//Lora 
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
// save transmission state between loops
static int transmissionState = RADIOLIB_ERR_NONE;
// flag to indicate that a packet was sent
static volatile bool transmittedFlag = false;
static uint32_t counter = 0;
static String loraPayload;

//TMP117 functions



// this function is called when a complete packet is transmitted
void setFlag(void)
{
    transmittedFlag = true;
}



struct BoardInfo {
  uint8_t id;
  uint8_t batteryPercent;
  float temp;
  int avgBPM;
  String address;
  uint8_t addressType;
  bool isSelf;
};

struct ReceivedPacket {
  bool valid;
  uint8_t id;
  uint8_t batteryPercent;
  float temp;
  int avgBPM;
  String address;
  String payload;
  uint32_t timeReceivedMs;
};

ReceivedPacket receivedPackets[MAX_NUM_BOARDS];


BoardInfo boards[MAX_NUM_BOARDS];
int boardCount = 0;

enum class CycleState {
    START_CYCLE,
    SCANNING,
    ELECT_LEADER,
    FOLLOWER_WAIT,
    LEADER_WAIT,
    LORA_TRANSMITTING,
    WAIT_FOR_NEXT_CYCLE
};

CycleState cycleState = CycleState::START_CYCLE;

uint32_t cycleStartMs = 0;
uint32_t stateStartMs = 0;

volatile bool scanFinished = false;
bool loraTransmitActive = false;

BoardInfo selectedLeader;

uint8_t knownBoardIds[MAX_NUM_BOARDS] = {
  0,
  1
};

bool isKnownBoard (uint8_t id) {
  for(int i = 0; i < MAX_NUM_BOARDS; i++){
    if(id == knownBoardIds[i]){
      return true;
    }
  }
  return false;
}

//Globals for this board
uint8_t myId = THIS_BOARD_ID;
String myName;

uint8_t myBatteryPercent = 0;
int myAvgBPM = 0;

uint32_t sampleCounter = 0;
bool hrReady;
NimBLECharacteristic *rxCharacteristic = nullptr;

//Battery functions
uint8_t getBatteryPercent(){
  int percent = 0;
  if(USE_FAKE_BATTERY){
    if(myId == 0){
      percent = 70;
    }
    if(myId == 1){
      percent = 85;
    }
  }
  else{
    if(PMU && PMU->isBatteryConnect()){
      percent = PMU->getBatteryPercent();
    }
  }
  return (uint8_t)percent;
}

void updateMyBatteryValues() {
  myBatteryPercent = getBatteryPercent();
  // myBatteryMv = getBatteryVoltageMv();
}

//Board list functions
void clearBoardList(){
  boardCount = 0;
}

void addOrUpdateBoards(uint8_t id, uint8_t percentage, float temp, int avgBPM, String address, uint8_t addressType, bool isSelf){
  for(int i = 0; i < boardCount; i++){    //if board count is 0, this for loop does not run
    if(boards[i].id == id){   //skip boards already discovered
      boards[i].batteryPercent = percentage;
      boards[i].temp = temp;
      boards[i].avgBPM = avgBPM;
      boards[i].address = address;
      boards[i].addressType = addressType;
      boards[i].isSelf = isSelf;
      return;
    }
  }

  if(boardCount < MAX_NUM_BOARDS){
    boards[boardCount].id = id;
    boards[boardCount].batteryPercent = percentage;
    boards[boardCount].temp = temp;
    boards[boardCount].avgBPM = avgBPM;
    boards[boardCount].address = address;
    boards[boardCount].addressType = addressType;
    boards[boardCount].isSelf = isSelf;
    boardCount++;
  }
}

void addSelftoBoardList(){
  addOrUpdateBoards(myId, myBatteryPercent, myTempF, myAvgBPM, "SELF", 0, true);
}

bool isBoardALeader(const BoardInfo& a, const BoardInfo& b){
  if(a.batteryPercent > b.batteryPercent){
    return true;
  }
  if(a.batteryPercent < b.batteryPercent){ 
    return false;
  }

  if(a.id > b.id){   //final tie breaker
    return true;
  }
  return false;
}

int findLeaderIndex(){

  if (boardCount == 0){
    return -1;
  }

  int bestIndex = 0;

  for(int i = 1; i < boardCount; i++){
    if(isBoardALeader(boards[i], boards[bestIndex])){
      bestIndex = i;
    }
  }
  return bestIndex;
}

bool parsePacket(String payload, uint8_t &id, uint8_t &percent, float &temp, int &avgBPM){
  int idIndex = payload.indexOf("id=");
  int percentIndex = payload.indexOf(",battPercent=");
  int tempIndex = payload.indexOf(",temp=");
  int avgBPMIndex = payload.indexOf(",avgBPM=");

  if (idIndex < 0 || percentIndex < 0 || tempIndex < 0 || avgBPMIndex < 0) {
    return false;
  }

  String idStr = payload.substring(idIndex + 3, percentIndex);
  String percentStr = payload.substring(percentIndex + 13, tempIndex);
  String tempStr = payload.substring(tempIndex + 6, avgBPMIndex);
  String avgBPMStr = payload.substring(avgBPMIndex + 8);

  id = (uint8_t)strtol(idStr.c_str(), nullptr, 16);  // because you send ID using HEX
  percent = (uint8_t)percentStr.toInt();
  temp = tempStr.toFloat();
  avgBPM = avgBPMStr.toInt();

  return true;
}

void storeReceivedPacket(String payload, String address){
  uint8_t id;
  uint8_t percent;
  float temp;
  int avgBPM;
  if(!parsePacket(payload, id, percent, temp, avgBPM)){
    Serial.println("Could not parse received BLE payload.");
    return;
  }
  for(int i = 0; i < MAX_NUM_BOARDS; i++){
    if(receivedPackets[i].valid && receivedPackets[i].id == id){   //update existing packet
      receivedPackets[i].batteryPercent = percent;
      receivedPackets[i].temp = temp;
      receivedPackets[i].avgBPM = avgBPM;
      receivedPackets[i].address = address;
      receivedPackets[i].payload = payload;
      receivedPackets[i].timeReceivedMs = millis();
      return;
    }
  }
  for(int i = 0; i < MAX_NUM_BOARDS; i++){   //new entry, store packet in first empty slot
    if(!receivedPackets[i].valid){
      receivedPackets[i].valid = true;
      receivedPackets[i].id = id;
      receivedPackets[i].batteryPercent = percent;
      receivedPackets[i].temp = temp;
      receivedPackets[i].avgBPM = avgBPM;
      receivedPackets[i].address = address;
      receivedPackets[i].payload = payload;
      receivedPackets[i].timeReceivedMs = millis();
      return;
    }
  }

}

//receive callback
class RxCallbacks: public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override{
    std::string value = pCharacteristic->getValue();
    String payload = value.c_str();
    String senderAddress = connInfo.getAddress().toString().c_str();

    Serial.println("Received from: ");
    Serial.println(senderAddress);

    Serial.println("Payload:");
    Serial.println(payload);

    storeReceivedPacket(payload, senderAddress);
  }
};

RxCallbacks rxCallbacks;

void setupBleServer()
{
    NimBLEServer *server = NimBLEDevice::createServer();
    server->advertiseOnDisconnect(true);
    NimBLEService *service = server->createService(SERVICE_UUID);

    rxCharacteristic = service->createCharacteristic(
        RX_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );

    rxCharacteristic->setCallbacks(&rxCallbacks);

    service->start();

    Serial.println("BLE server started.");
}

//BLE advertising
void updateAdvertisement() {
  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->stop();
  NimBLEAdvertisementData advData;

  advData.setName(myName.c_str());

  //data format: byte 0: 'T', byte 1: 'B', byte 2: board ID high byte, byte 3: board ID low byte, byte 4: battery percent, byte 5: battery voltage high byte, byte 6: battery voltage low byte

  int16_t tempHundredths = 0;
  if (tempValid) {
      tempHundredths =
          static_cast<int16_t>(roundf(myTempF * 100.0f));
  }

  uint16_t tempBits =
        static_cast<uint16_t>(tempHundredths);

  uint8_t bpmByte =
    static_cast<uint8_t>(
        constrain(myAvgBPM, 0, 255)
    );


  std::string data;
  data.push_back('T');
  data.push_back('B');
  data.push_back(myId);
  data.push_back((char)myBatteryPercent);
  data.push_back(static_cast<char>(tempBits >> 8));
  data.push_back(static_cast<char>(tempBits & 0xFF));
  data.push_back(static_cast<char>(bpmByte));

  advData.setManufacturerData(data);

  advertising->setAdvertisementData(advData);
  advertising->start();



}

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* device) override{
    //check if advertised device has appropriate data
    if(!device->haveManufacturerData()){
      return;
    }

    std::string data = device->getManufacturerData();   //get data from advertised board
    if(data.length() < 7){
      return;
    }

    if(data[0] != 'T' || data[1] != 'B'){
      return;
    }

    uint8_t peerId = data[2];

    if(peerId == myId){
      return;
    }

    if (!isKnownBoard(peerId)) {
      return;
    }

    uint8_t peerBatteryPercent = (uint8_t)data[3];

    uint16_t tempBits =
    (static_cast<uint16_t>(
        static_cast<uint8_t>(data[4])) << 8) |
     static_cast<uint8_t>(data[5]);

    int16_t peerTempHundredths =
    static_cast<int16_t>(tempBits);

    float peerTempF = 0.0f;

    
    peerTempF = peerTempHundredths / 100.0f;
    uint8_t peerAvgBPM =
    static_cast<uint8_t>(data[6]);

    String peerAddress = device->getAddress().toString().c_str();
    uint8_t peerAddressType = device->getAddressType();

    addOrUpdateBoards(peerId, peerBatteryPercent, peerTempF, peerAvgBPM, peerAddress, peerAddressType, false);

  }
  void onScanEnd(const NimBLEScanResults &results, int reason) override{
    scanFinished = true;
    Serial.println("BLE scan complete.");

    Serial.print("Boards found including self: ");
    Serial.println(boardCount);
    Serial.print("BLE scan ended, reason: ");
    Serial.println(reason);
  }
};

ScanCallbacks scanCallbacks;

void setUpScanner(){
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&scanCallbacks, false);

  scan->setActiveScan(false);
  scan->setInterval(100);
  scan->setWindow(50);
  scan->setMaxResults(0);
  Serial.println("BLE scanner initialized");
}

bool startBoardScan(){
  clearBoardList();
  addSelftoBoardList();
  
  scanFinished = false;
  NimBLEScan *scan = NimBLEDevice::getScan();
  Serial.println("Starting nonblocking BLE scan.");
  

  return scan->start(SCAN_TIME_MS, false, true); //scan for SCAN_TIME_MS, does not clear the old scan results before starting, will restart scan if scan is already active
}

//send data to leader via Bluetooth
String makePayload(){
  String payload = "";
  payload += "id=";
  payload += String(myId, HEX);
  payload += ",battPercent=";
  payload += String(myBatteryPercent);
  payload += ",temp=";
  payload += String(myTempF, 2);
  payload += ",avgBPM=";
  payload += String(myAvgBPM);
  return payload;

}

bool sendDatatoLeader(const BoardInfo& leader){
    Serial.print("Leader ID: 0x");
    Serial.println(leader.id, HEX);

    Serial.print("Leader Address: ");
    Serial.println(leader.address);

    NimBLEClient *client = NimBLEDevice::createClient();

    if (client == nullptr) {
        Serial.println("Failed to create BLE client.");
        return false;
    }

    NimBLEAddress leaderAddress(
      std::string(leader.address.c_str()),
      leader.addressType
    );

    if (!client->connect(leaderAddress)) {
        Serial.println("Failed to connect to leader.");
        NimBLEDevice::deleteClient(client);
        return false;
    }

    Serial.println("Connected to leader.");

    NimBLERemoteService *remoteService = client->getService(SERVICE_UUID);
    if(remoteService == nullptr){
      Serial.println("Failed to connect to leader service");
      client->disconnect();
      NimBLEDevice::deleteClient(client);
      return false;
    }

    NimBLERemoteCharacteristic *remoteRx = remoteService->getCharacteristic(RX_UUID);  //find leaders RX characteristic so this board can write to it

    if (remoteRx == nullptr) {
        Serial.println("Leader RX characteristic not found.");
        client->disconnect();
        NimBLEDevice::deleteClient(client);
        return false;
    }
    String payload = makePayload();
    

    bool success = remoteRx->writeValue(
        (uint8_t *)payload.c_str(),
        payload.length(),
        true
    );

    if(success){
      Serial.println("sent to leader: ");
      Serial.print(payload);
    }
    else{
      Serial.println("Could not send to leader");
    }

    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return success;
}



//Lora 
void setupLora(){
  #ifdef RADIO_TCXO_ENABLE
    pinMode(RADIO_TCXO_ENABLE, OUTPUT);
    digitalWrite(RADIO_TCXO_ENABLE, HIGH);
#endif

    // initialize radio with default settings
    int state = radio.begin();

    printResult(state == RADIOLIB_ERR_NONE);

    Serial.printf("[%s]:", RADIO_TYPE_STR);
    Serial.print(F("Radio Initializing ... "));

    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }

    // set the function that will be called when packet transmission is finished
    radio.setPacketSentAction(setFlag);

    // SX1262 allowed frequency range: 150.0 MHz to 960.0 MHz
    if (radio.setFrequency(CONFIG_RADIO_FREQ) == RADIOLIB_ERR_INVALID_FREQUENCY) {
        Serial.println(F("Selected frequency is invalid for this module!"));
        while (true);
    }

    // SX1262 allowed bandwidths:
    // 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0, 500.0 kHz
    if (radio.setBandwidth(CONFIG_RADIO_BW) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
        Serial.println(F("Selected bandwidth is invalid for this module!"));
        while (true);
    }

    // SX1262 spreading factor allowed range: 5 to 12
    if (radio.setSpreadingFactor(12) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
        Serial.println(F("Selected spreading factor is invalid for this module!"));
        while (true);
    }

    // SX1262 coding rate denominator allowed range: 5 to 8
    if (radio.setCodingRate(6) == RADIOLIB_ERR_INVALID_CODING_RATE) {
        Serial.println(F("Selected coding rate is invalid for this module!"));
        while (true);
    }

    // Set LoRa sync word
    if (radio.setSyncWord(0xAB) != RADIOLIB_ERR_NONE) {
        Serial.println(F("Unable to set sync word!"));
        while (true);
    }

    // SX1262 output power allowed range: -9 dBm to 22 dBm
    if (radio.setOutputPower(CONFIG_RADIO_OUTPUT_POWER) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        Serial.println(F("Selected output power is invalid for this module!"));
        while (true);
    }

    // SX1262 current limit:
    // 45 to 120 mA in 2.5 mA steps,
    // 120 to 240 mA in 10 mA steps.
    // Set to 0 to disable overcurrent protection.
    if (radio.setCurrentLimit(140) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
        Serial.println(F("Selected current limit is invalid for this module!"));
        while (true);
    }

    // SX1262 preamble length allowed range: 1 to 65535
    if (radio.setPreambleLength(16) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH) {
        Serial.println(F("Selected preamble length is invalid for this module!"));
        while (true);
    }

    // Enable or disable CRC check
    if (radio.setCRC(false) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
        Serial.println(F("Selected CRC is invalid for this module!"));
        while (true);
    }

#ifdef USING_DIO2_AS_RF_SWITCH
    // Some SX126x modules use DIO2 as RF switch.
    // As long as DIO2 is configured to control RF switch,
    // it cannot be used as interrupt pin.
    if (radio.setDio2AsRfSwitch() != RADIOLIB_ERR_NONE) {
        Serial.println(F("Failed to set DIO2 as RF switch!"));
        while (true);
    }
#endif

#ifdef RADIO_CTRL
    Serial.println("Turn off LAN, Turn on PA, Enter TX mode.");

    /*
       2W LoRa LAN Control:
       Set LOW to turn off LAN and enter TX mode.
    */
    digitalWrite(RADIO_CTRL, LOW);
#endif

    // start transmitting the first packet
    // Serial.print(F("Radio Sending first packet ... "));

    // transmissionState = radio.startTransmit(String(counter).c_str());

    delay(1000);
}


void clearReceivedPackets(){
  for(int i = 0; i < MAX_NUM_BOARDS; i++){
    receivedPackets[i] = {};
  }
}


void buildLoraPayload(){
  loraPayload = "";
  loraPayload += makePayload();

  for(int i = 0; i < MAX_NUM_BOARDS; i++){
    if(receivedPackets[i].valid){
      loraPayload += ";";
      loraPayload += receivedPackets[i].payload;      
    }
  }
  Serial.print("LoRa payload: ");
  Serial.println(loraPayload);
}

void runFSM(){
  uint32_t now = millis();

  switch (cycleState){
    case CycleState::START_CYCLE:
    {
      cycleStartMs = now;
      clearReceivedPackets();
      updateMyBatteryValues();
      if(tmp117Ready){
        tempValid = tempSensor.readTemp(myTempF);
        if(tempValid){
          Serial.print("Temperature: ");
          Serial.print(myTempF);
          Serial.println(" F");
        }
        else{
          Serial.println("Temperature read failed");
        }
      }
      else{
        tempValid = false;
      }
      updateAdvertisement(); 
      if(!startBoardScan()){
        Serial.println("Failed to start scan.");
        cycleState = CycleState::WAIT_FOR_NEXT_CYCLE;
        break;
      }
      cycleState = CycleState::SCANNING;
      break;
    }
    case CycleState::SCANNING:
    {
      if(!scanFinished){
        break;
      }
      Serial.print("Boards found: "); 
      Serial.println(boardCount);

      cycleState = CycleState::ELECT_LEADER;
      break;
    }
    case CycleState::ELECT_LEADER:
    {
      if (boardCount < MAX_NUM_BOARDS) {
        Serial.println("Not all expected boards were found. Skipping this cycle.");
        cycleState = CycleState::WAIT_FOR_NEXT_CYCLE;
        break;
      }

      int leaderIndex = findLeaderIndex();

      if(leaderIndex < 0){
        Serial.println("No leader found");
        cycleState = CycleState::WAIT_FOR_NEXT_CYCLE;
        break;
      }
      selectedLeader = boards[leaderIndex];
      stateStartMs = now;

      if(selectedLeader.isSelf){
        Serial.println("I am the highest-battery board. Waiting to receive data.");
        cycleState = CycleState::LEADER_WAIT;
      }
      else{
        Serial.println( "I am follower; waiting for send slot." );
        cycleState = CycleState::FOLLOWER_WAIT;
      }
      break;
    }
    case CycleState::FOLLOWER_WAIT:
    {
      uint32_t sendDelay = 500 + (myId % 5) * 1000;
      if(now - stateStartMs < sendDelay){
        break;
      }
      sendDatatoLeader(selectedLeader);
      cycleState = CycleState::WAIT_FOR_NEXT_CYCLE;
      break;
    }
    case CycleState::LEADER_WAIT:
    {
      if(now - stateStartMs < LEADER_RECEIVE_MS){
        break;
      }
      NimBLEDevice::getAdvertising()->stop();
      buildLoraPayload();
      transmittedFlag = false;
      int transmissionState = radio.startTransmit(loraPayload.c_str());
      
      if (transmissionState == RADIOLIB_ERR_NONE) {
        Serial.println("LoRa transmit started.");
      } 
      else {
        Serial.print("LoRa transmit failed, code ");
        Serial.println(transmissionState);
        cycleState = CycleState::WAIT_FOR_NEXT_CYCLE;
        break;
      }
      loraTransmitActive = true;
      cycleState = CycleState::LORA_TRANSMITTING;
      break;
    }
    case CycleState::LORA_TRANSMITTING:
    {
      if(!transmittedFlag){
        break;
      }
      transmittedFlag = false;
      loraTransmitActive = false;
      int state = radio.finishTransmit();
      if (state == RADIOLIB_ERR_NONE) {
        Serial.println("LoRa transmit success.");
      } else {
        Serial.print("LoRa finish failed, code ");
        Serial.println(state);
      }
      cycleState = CycleState::WAIT_FOR_NEXT_CYCLE;
      break;
    }
    case CycleState::WAIT_FOR_NEXT_CYCLE:
    {
      if (now - cycleStartMs < CYCLE_TIME_MS) {
         break; 
      }
      cycleState = CycleState::START_CYCLE;
      break;
    }
  }
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  setupBoards();
  updateMyBatteryValues();
      /*
       Initialize Wire only once.

       If setupBoards() already initializes Wire for the PMU,
       do not call Wire.begin() again. You can just set the clock.
    */

  Wire.setClock(100000);

  tmp117Ready = tempSensor.begin();
  if(!tmp117Ready){
    Serial.println("Failed to initialize temp sensor");
  } else{
    Serial.println("Temp sensor initialized");
  }

  hrReady = hrSensor.begin();
  if (hrReady) {
    Serial.println("MAX30102 initialized.");
} else {
    Serial.println("MAX30102 initialization failed.");
}

  myName = "Board_";
  myName += String(myId);
   
  NimBLEDevice::init(myName.c_str());

  setupBleServer();
  setUpScanner();
  setupLora();

}

void loop() {
  // put your main code here, to run repeatedly:
  if(hrReady){
    hrSensor.update();
    myAvgBPM = hrSensor.getAverageBPM();
  }
  else{
    myAvgBPM = 0;
  }
  
  
  runFSM();
  delay(1);
}

