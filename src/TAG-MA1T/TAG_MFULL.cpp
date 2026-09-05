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

// ---- CIR Extras ----
#define ACC_MEM 0x25
#define CIR_NUM_SAMPLES 1016
#define CIR_BUF_LEN (1 + CIR_NUM_SAMPLES * 4)
#define LDE_IF 0x2E
#define LDE_PPINDX_SUB 0x1000
#define LDE_PPAMPL_SUB 0x1002
#define LEN_LDE_PPINDX 2
#define LEN_LDE_PPAMPL 2

volatile byte expectedMsgId = POLL_ACK;
volatile boolean sentAck = false;
volatile boolean receivedAck = false;

DW1000Time timePollSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent; 

#define LEN_DATA 17
byte data[LEN_DATA]; 

uint32_t lastActivity;
uint32_t resetPeriod = 250;
uint16_t replyDelayTimeUS = 7000;  
const uint32_t ROUND_DELAY_MS = 500; 

static constexpr uint16_t ANTENNA_DELAY = 16497;
static constexpr uint8_t UWB_CHANNEL = DW1000.CHANNEL_2;
static constexpr uint8_t UWB_PREAMBLE_CODE = DW1000.PREAMBLE_CODE_64MHZ_10;
uint32_t nextRoundAt = 0; 
boolean roundInProgress = false; 

const uint8_t NUM_ANCHORS = 4; 
uint8_t anchorIDs[NUM_ANCHORS] = {2,3,4,5}; 
uint8_t currentTargetIndex = 0; 

uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;
uint32_t cirSeq = 0;
byte cirBuffer[CIR_BUF_LEN];
void noteActivity(); 
void handleSent();
void receiver(); 
void handleReceived(); 
void startRound(); 
void transmitPoll(); 
// ---- CIR SPI Functions ----
void enableAccClock(bool on) {
    byte reg[2];
    DW1000.readBytes(PMSC, PMSC_CTRL0_SUB, reg, 2);
    if (on) {
        reg[0] = 0x48 | (reg[0] & 0xb3);
        reg[1] = 0x80 | reg[1];
    } else {
        reg[0] = reg[0] & 0xb3;
        reg[1] = 0x7f & reg[1];
    }
    DW1000.writeByte(PMSC, PMSC_CTRL0_SUB, reg[0]);
    DW1000.writeByte(PMSC, PMSC_CTRL0_SUB + 1, reg[1]);
}

void getAccumulator(byte* out, uint16_t numSamples, uint16_t accOffset = 0) {
    enableAccClock(true);
    DW1000.readBytes(ACC_MEM, accOffset, out, numSamples * 4 + 1);
    enableAccClock(false);
}

uint16_t readStdNoise() {
    byte buf[LEN_STD_NOISE];
    DW1000.readBytes(RX_FQUAL, STD_NOISE_SUB, buf, LEN_STD_NOISE);
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

void readFpIndexAndAmpl1(uint16_t &fpIndex, uint16_t &fpAmpl1) {
    byte buf[LEN_RX_TIME];
    DW1000.readBytes(RX_TIME, 0, buf, LEN_RX_TIME);
    fpIndex = (uint16_t)buf[5] | ((uint16_t)buf[6] << 8);
    fpAmpl1 = (uint16_t)buf[7] | ((uint16_t)buf[8] << 8);
}

uint16_t readFpAmpl2() {
    byte buf[LEN_FP_AMPL2];
    DW1000.readBytes(RX_FQUAL, FP_AMPL2_SUB, buf, LEN_FP_AMPL2);
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

uint16_t readFpAmpl3() {
    byte buf[LEN_FP_AMPL3];
    DW1000.readBytes(RX_FQUAL, FP_AMPL3_SUB, buf, LEN_FP_AMPL3);
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

uint16_t readRxPacc() {
    byte rxFrameInfo[LEN_RX_FINFO];
    DW1000.readBytes(RX_FINFO, NO_SUB, rxFrameInfo, LEN_RX_FINFO);
    return (((uint16_t)rxFrameInfo[2] >> 4) & 0xFF) | ((uint16_t)rxFrameInfo[3] << 4);
}

uint16_t readLdePpIndx() {
    byte buf[LEN_LDE_PPINDX];
    DW1000.readBytes(LDE_IF, LDE_PPINDX_SUB, buf, LEN_LDE_PPINDX);
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

uint16_t readLdePpAmpl() {
    byte buf[LEN_LDE_PPAMPL];
    DW1000.readBytes(LDE_IF, LDE_PPAMPL_SUB, buf, LEN_LDE_PPAMPL);
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

void captureAndDumpCir(float distance) {
    cirSeq++;
    uint16_t stdNoise = readStdNoise();
    uint16_t fpIndex, fpAmpl1;
    readFpIndexAndAmpl1(fpIndex, fpAmpl1);
    uint16_t fpAmpl2 = readFpAmpl2();
    uint16_t fpAmpl3 = readFpAmpl3();
    uint16_t rxpacc = readRxPacc();
    uint16_t ldePpIndx = readLdePpIndx();
    uint16_t ldePpAmpl = readLdePpAmpl();
    float rxPower = DW1000.getReceivePower();
    float fpPower = DW1000.getFirstPathPower();

    getAccumulator(cirBuffer, CIR_NUM_SAMPLES, 0);

    DebugSerial.print(F("RANGE,"));
    DebugSerial.print(cirSeq);                        DebugSerial.print(',');
    DebugSerial.print(distance, 4);                   DebugSerial.print(',');
    DebugSerial.print(rxPower, 4);                    DebugSerial.print(',');
    DebugSerial.print(anchorIDs[currentTargetIndex]); DebugSerial.print(','); 
    DebugSerial.println(samplingRate);

    DebugSerial.print(F("CIR,"));
    DebugSerial.print(cirSeq);           DebugSerial.print(',');
    DebugSerial.print(fpIndex);          DebugSerial.print(',');
    DebugSerial.print(ldePpIndx);        DebugSerial.print(',');
    DebugSerial.print(ldePpAmpl);        DebugSerial.print(',');
    DebugSerial.print(fpAmpl1);          DebugSerial.print(',');
    DebugSerial.print(fpAmpl2);          DebugSerial.print(',');
    DebugSerial.print(fpAmpl3);          DebugSerial.print(',');
    DebugSerial.print(stdNoise);         DebugSerial.print(',');
    DebugSerial.print(rxpacc);           DebugSerial.print(',');
    DebugSerial.print(rxPower, 4);       DebugSerial.print(',');
    DebugSerial.print(fpPower, 4);       DebugSerial.print(',');
    DebugSerial.print(CIR_NUM_SAMPLES);  DebugSerial.print(',');
    for (uint16_t i = 1; i < CIR_BUF_LEN; i++) {
        if (cirBuffer[i] < 0x10) DebugSerial.print('0');
        DebugSerial.print(cirBuffer[i], HEX);
    }
    DebugSerial.println();
}

void setup(){
    DebugSerial.begin(921600);
    delay(1000);
    DebugSerial.println(F("### Ranging Multi Tag CIR ###"));

    DW1000.begin(DW_IRQ, DW_RST);
    DW1000.select(DW_CS);

    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(1); 
    DW1000.setNetworkId(10); 
    DW1000.setAntennaDelay(ANTENNA_DELAY);  
    DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY);
    DW1000.setChannel(UWB_CHANNEL);
    DW1000.setPreambleCode(UWB_PREAMBLE_CODE); 
    DW1000.commitConfiguration(); 

    DW1000.attachSentHandler(handleSent);
    DW1000.attachReceivedHandler(handleReceived);

    receiver();
    startRound();
    noteActivity();
}

void noteActivity() { lastActivity = millis(); }

void resetInactive(){
    expectedMsgId = POLL_ACK;
    roundInProgress = false; 
    currentTargetIndex = (currentTargetIndex + 1) % NUM_ANCHORS; // Advance on timeout
    nextRoundAt = millis() + ROUND_DELAY_MS; 
    
    // Force the hardware to flush any corrupted RF states
    DW1000.receivePermanently(true); 
    receiver(); 
    
    noteActivity(); 
}

void startRound() {
    roundInProgress = true;
    expectedMsgId = POLL_ACK;
    transmitPoll();
}

void handleSent() { sentAck = true; }

void handleReceived() {
    DW1000.getData(data, LEN_DATA); 
    // Tag only extracts CIR on the final RANGE_REPORT from Anchor
    if (data[0] == RANGE_REPORT){
        DW1000.receivePermanently(false); // Freeze receiver to protect CIR
    }
    receivedAck = true;
}

void transmitPoll() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL;
    data[1] = anchorIDs[currentTargetIndex]; 
    DebugSerial.printf("Polling to anchor %d\n", (int)data[1]); 
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void transmitRange() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE;
    data[1] = anchorIDs[currentTargetIndex]; 
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    timeRangeSent = DW1000.setDelay(deltaTime);
    timePollSent.getTimestamp(data + 2);
    timePollAckReceived.getTimestamp(data + 7);
    timeRangeSent.getTimestamp(data + 12);
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
        if (msgId == POLL) DW1000.getTransmitTimestamp(timePollSent);
        else if (msgId == RANGE) DW1000.getTransmitTimestamp(timeRangeSent);
    }

    if (receivedAck) {
        receivedAck = false;
        DW1000.getData(data, LEN_DATA); 
        byte msgId = data[0];
        
        if (msgId != expectedMsgId) {
            resetInactive();
            return;
        }
        
        if (msgId == POLL_ACK) {
            if (data[1] != anchorIDs[currentTargetIndex]) return;
            DW1000.getReceiveTimestamp(timePollAckReceived);
            expectedMsgId = RANGE_REPORT;
            transmitRange();
            noteActivity();
            
        } else if (msgId == RANGE_REPORT) {
            if (data[1] != anchorIDs[currentTargetIndex]) return; 
            float curRange;
            memcpy(&curRange, data + 2, 4);
            
            // 1. Safe CIR Extraction
            captureAndDumpCir(curRange);

            // 2. Re-enable the library flag and physically restart the radio hardware
            DW1000.receivePermanently(true); //[cite: 1]
            receiver(); // Executes startReceive() to write to SYS_CTRL[cite: 1]
            
            // 3. Close round and queue next Anchor
            expectedMsgId = POLL_ACK;
            roundInProgress = false;
            currentTargetIndex = (currentTargetIndex + 1) % NUM_ANCHORS;
            nextRoundAt = millis() + ROUND_DELAY_MS; 
            noteActivity();
            
        } else if (msgId == RANGE_FAILED) {
            resetInactive(); 
        }
    }
}