#include <SPI.h> 
#include <DW1000.h> 
#include <Arduino.h> 

static constexpr uint8_t DW_CS  = PA4;
static constexpr uint8_t DW_IRQ = PB0;
static constexpr uint8_t DW_RST = PB12;

HardwareSerial DebugSerial(PA10, PA9); 

#define POLL 0 
#define POLL_ACK 1 
#define RANGE 2 
#define RANGE_REPORT 3 
#define RANGE_FAILED 255 

volatile byte expectedMsgId = POLL_ACK;
volatile boolean sentAck = false;
volatile boolean receivedAck = false;

DW1000Time timePollSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent; 


#define LEN_DATA 16
byte data[LEN_DATA]; 

uint32_t lastActivity;
uint32_t resetPeriod = 250;
uint16_t replyDelayTimeUS = 7000;  
const uint32_t ROUND_DELAY_MS = 1000; 

static constexpr uint16_t ANTENNA_DELAY = 16497;
//The longest, most durable signal range 
static constexpr uint8_t UWB_CHANNEL = DW1000.CHANNEL_2;
static constexpr uint8_t UWB_PREAMBLE_CODE = DW1000.PREAMBLE_CODE_64MHZ_10;
uint32_t nextRoundAt = 0; 
boolean roundInProgress = false; 
void noteActivity(); 
void handleSent();
void receiver(); 
void handleReceived(); 
void startRound(); 
void transmitPoll(); 
void setup(){
    DebugSerial.begin(921600);
    delay(1000);
    DebugSerial.println(F("### RangingTagCIR ###"));

    DW1000.begin(DW_IRQ, DW_RST);
    DW1000.select(DW_CS);
    DebugSerial.println(F("DW1000 initialized ..."));

    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(2); 
    DW1000.setNetworkId(10); 
    DW1000.setAntennaDelay(ANTENNA_DELAY);  
    //DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER); 
    DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY);  // sets PRF to 64MHz
    DW1000.setChannel(UWB_CHANNEL);
    DW1000.setPreambleCode(UWB_PREAMBLE_CODE);   // now matches 64MHz PRF
    DW1000.commitConfiguration();
    
    DW1000.commitConfiguration(); 
    DebugSerial.println(F("Committed configuration ...")); 

    DW1000.attachSentHandler(handleSent);
    DW1000.attachReceivedHandler(handleReceived);

    receiver();
    startRound();
    noteActivity();
}

void noteActivity() {
    lastActivity = millis(); 
}

void resetInactive() {
    expectedMsgId = POLL_ACK; 
    roundInProgress = false; 
    nextRoundAt = millis() + ROUND_DELAY_MS;
    noteActivity();  
}

void startRound() {
    roundInProgress = true;
    transmitPoll();
}

void handleSent() {
    sentAck = true;
}

void handleReceived() {
    receivedAck = true;
}

void transmitPoll() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL;
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}


void transmitRange() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE;
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    timeRangeSent = DW1000.setDelay(deltaTime);
    timePollSent.getTimestamp(data + 1);
    timePollAckReceived.getTimestamp(data + 6);
    timeRangeSent.getTimestamp(data + 11);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void receiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

void loop() {
    if (!sentAck && !receivedAck) {
        if (roundInProgress && millis() - lastActivity > resetPeriod) {
            resetInactive();
        } else if (!roundInProgress && millis() >= nextRoundAt) {
            startRound();
            noteActivity();
        }
        return;
    }

    if (sentAck) {
        sentAck = false;
        byte msgId = data[0];
        if (msgId == POLL) {
            DW1000.getTransmitTimestamp(timePollSent);
        } else if (msgId == RANGE) {
            DW1000.getTransmitTimestamp(timeRangeSent);
            noteActivity();
        }
    }

    if (receivedAck) {
        receivedAck = false;
        DW1000.getData(data, LEN_DATA);
        byte msgId = data[0];
        if (msgId != expectedMsgId) {
            // unexpected message -- end this round, wait, retry
            DebugSerial.println("Wrong Messages");
            resetInactive();
            return;
        }
        if (msgId == POLL_ACK) {
            DW1000.getReceiveTimestamp(timePollAckReceived);
            expectedMsgId = RANGE_REPORT;
            transmitRange();
            noteActivity();
        } else if (msgId == RANGE_REPORT) {
            float curRange;
            memcpy(&curRange, data + 1, 4);
            DebugSerial.print(F("Range: ")); DebugSerial.print(curRange); DebugSerial.println(F(" m"));
            expectedMsgId = POLL_ACK;
            roundInProgress = false;
            nextRoundAt = millis() + ROUND_DELAY_MS;
            noteActivity();
        } else if (msgId == RANGE_FAILED) {
            expectedMsgId = POLL_ACK;
            roundInProgress = false;
            nextRoundAt = millis() + ROUND_DELAY_MS;
            noteActivity();
        }
    }
}