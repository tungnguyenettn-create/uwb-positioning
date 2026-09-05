#include <SPI.h>
#include <DW1000.h>
#include <Arduino.h> 

static constexpr uint8_t DW_CS  = PA4;
static constexpr uint8_t DW_IRQ = PB0;
static constexpr uint8_t DW_RST = PB12;
HardwareSerial DebugSerial(PA10, PA9);

#define ANCHOR_ID 3 // Change for each Anchor (2, 3, 4, 5)

#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255

volatile byte expectedMsgId = POLL;
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
boolean protocolFailed = false;

DW1000Time timePollSent;
DW1000Time timePollReceived;
DW1000Time timePollAckSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
DW1000Time timeRangeReceived;
DW1000Time timeComputedRange;

#define LEN_DATA 17
byte data[LEN_DATA];

uint32_t lastActivity;
uint32_t resetPeriod = 250;
uint16_t replyDelayTimeUS = 7000;  

static constexpr uint16_t ANTENNA_DELAY = 16497;
static constexpr uint8_t UWB_CHANNEL = DW1000.CHANNEL_2;
static constexpr uint8_t UWB_PREAMBLE_CODE = DW1000.PREAMBLE_CODE_64MHZ_10; 
void noteActivity(); 
void handleSent();
void receiver(); 
void handleReceived(); 
void setup(){
    DebugSerial.begin(921600);
    delay(1000);
    DebugSerial.println(F("### Lightweight Ranging Anchor ###"));

    DW1000.begin(DW_IRQ, DW_RST);
    DW1000.select(DW_CS);

    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(ANCHOR_ID);
    DW1000.setNetworkId(10); 
    DW1000.setAntennaDelay(ANTENNA_DELAY); 
    DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY);  
    DW1000.setChannel(UWB_CHANNEL);
    DW1000.setPreambleCode(UWB_PREAMBLE_CODE);   
    DW1000.commitConfiguration(); 

    DW1000.attachSentHandler(handleSent);
    DW1000.attachReceivedHandler(handleReceived);

    receiver();
    noteActivity();
}

void noteActivity() {
    lastActivity = millis();
}

void resetInactive() {
    expectedMsgId = POLL; 
    receiver(); 
    noteActivity();
}

void handleSent() { sentAck = true; }
void handleReceived() { receivedAck = true; }

void receiver() {
    DW1000.newReceive(); 
    DW1000.setDefaults(); 
    DW1000.receivePermanently(true); 
    DW1000.startReceive(); 
}

void transmitPollAck() {
    DW1000.newTransmit();
    DW1000.setDefaults(); 
    data[0] = POLL_ACK; 
    data[1] = ANCHOR_ID;
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    DW1000.setDelay(deltaTime);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit(); 
}

void transmitRangeReport(float curRange) {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE_REPORT;
    data[1] = ANCHOR_ID;
    memcpy(data + 2, &curRange, 4);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void transmitRangeFailed() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE_FAILED;
    data[1] = ANCHOR_ID; 
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void computeRangeAsymmetric() {
    DW1000Time round1 = (timePollAckReceived - timePollSent).wrap();
    DW1000Time reply1 = (timePollAckSent - timePollReceived).wrap();
    DW1000Time round2 = (timeRangeReceived - timePollAckSent).wrap();
    DW1000Time reply2 = (timeRangeSent - timePollAckReceived).wrap();
    DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
    timeComputedRange.setTimestamp(tof);
}

void loop(){
    int32_t curMillis = millis(); 
    if (!sentAck && !receivedAck){
        if (curMillis - lastActivity > resetPeriod){
            resetInactive(); 
        }
        return; 
    }
    
    if (sentAck) {
        sentAck = false;
        byte msgId = data[0];
        if (msgId == POLL_ACK) {
            DW1000.getTransmitTimestamp(timePollAckSent);
            noteActivity();
        }
    }
    
    if (receivedAck) {
        receivedAck = false;
        DW1000.getData(data, LEN_DATA);
        byte msgId = data[0];
        

        if ((msgId == POLL || msgId == RANGE) && data[1] != ANCHOR_ID) {
            DebugSerial.print("Wrong Message\n"); 

            return;
        }

        if (msgId != expectedMsgId) {
            protocolFailed = true;
        }
        
        if (msgId == POLL) {
            DebugSerial.print("Received Polling\n");
            protocolFailed = false;
            DW1000.getReceiveTimestamp(timePollReceived);
            expectedMsgId = RANGE;
            DebugSerial.print("Sent POLL ACK\n");
            transmitPollAck();
            noteActivity();
        }
        else if (msgId == RANGE) {
            DebugSerial.print("Received RANGE\n");
            DW1000.getReceiveTimestamp(timeRangeReceived);
            expectedMsgId = POLL;
            
            if (!protocolFailed) {
                timePollSent.setTimestamp(data + 2);
                timePollAckReceived.setTimestamp(data + 7);
                timeRangeSent.setTimestamp(data + 12);
                
                computeRangeAsymmetric(); 
                float curRange = timeComputedRange.getAsMicroSeconds(); 
                DebugSerial.print("Range: ");
                DebugSerial.print(curRange, 4);   // 4 decimal places
                DebugSerial.println();
                DebugSerial.print("Sent RANGE REPORT\n");
                transmitRangeReport(curRange);
                
            } else {
                transmitRangeFailed();
            }
            noteActivity();
        }
    }
}