#include <SPI.h>
#include <DW1000.h>
#include <Arduino.h>
// connection pins
static constexpr uint8_t DW_CS  = PA4;
static constexpr uint8_t DW_IRQ = PB0;
static constexpr uint8_t DW_RST = PB12;
HardwareSerial DebugSerial(PA10, PA9);


// messages used in the ranging protocol
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255

// ---- CIR / accumulator / LDE extras (not in DW1000Constants.h) ----
#define ACC_MEM 0x25
#define CIR_NUM_SAMPLES 1016   // 64 MHz PRF max (MODE_LONGDATA_RANGE_ACCURACY, set below)
//#define CIR_NUM_SAMPLES 992 
#define CIR_BUF_LEN (1 + CIR_NUM_SAMPLES * 4)
#define LDE_IF 0x2E
#define LDE_PPINDX_SUB 0x1000
#define LDE_PPAMPL_SUB 0x1002
#define LEN_LDE_PPINDX 2
#define LEN_LDE_PPAMPL 2

// message flow state
volatile byte expectedMsgId = POLL;
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
boolean protocolFailed = false;

// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollReceived;
DW1000Time timePollAckSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
DW1000Time timeRangeReceived;
DW1000Time timeComputedRange;

#define LEN_DATA 16
byte data[LEN_DATA];

uint32_t lastActivity;
uint32_t resetPeriod = 250;
uint16_t replyDelayTimeUS = 7000;  // stock value -- the round is no longer
                                    // latency-sensitive overall, but this
                                    // specific delay still governs the
                                    // POLL_ACK/RANGE delayed-TX margin

uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;
uint32_t cirSeq = 0;  // increments once per captured CIR, ties CIR line to RANGE line

byte cirBuffer[CIR_BUF_LEN];


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
    DebugSerial.println(F("### RangingAnchorCIR ###"));

    DW1000.begin(DW_IRQ, DW_RST);
    DW1000.select(DW_CS);
    DebugSerial.println(F("DW1000 initialized ..."));

    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(1); 
    DW1000.setNetworkId(10); 
    DW1000.setAntennaDelay(ANTENNA_DELAY); 
    DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY);  // sets PRF to 64MHz
    DW1000.setChannel(UWB_CHANNEL);
    DW1000.setPreambleCode(UWB_PREAMBLE_CODE);   // now matches 64MHz PRF
    
    //DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
    DW1000.commitConfiguration(); 
    DebugSerial.println(F("Committed configuration ...")); 

    DW1000.attachSentHandler(handleSent);
    DW1000.attachReceivedHandler(handleReceived);

    receiver();
    rangingCountPeriod = millis();
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

void handleSent() {
    sentAck = true;
}

void handleReceived() {
    // Peek the message type (cheap, 16-byte SPI read) so we can suppress
    // the library's auto re-arm specifically for RANGE receptions -- the
    // only point where we need the accumulator/diagnostic registers to
    // still be valid once loop() gets around to reading them. Every
    // other message (POLL etc.) behaves exactly like stock: auto
    // re-arm happens normally and is already proven safe there.
    DW1000.getData(data, LEN_DATA);
    if (data[0] == RANGE) {
        DW1000.receivePermanently(false);  // just a flag -- no SPI transaction
    }
    receivedAck = true;
}

void transmitPollAck() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL_ACK;
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    DW1000.setDelay(deltaTime);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void transmitRangeReport(float curRange) {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE_REPORT;
    memcpy(data + 1, &curRange, 4);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void transmitRangeFailed() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE_FAILED;
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void receiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    // stock behavior restored: permanentReceive(true) is the persistent
    // default. We no longer fight the library's own re-arm logic with
    // manual newReceive()/idle() calls (that was cancelling pending
    // delayed transmissions -- TRXOFF aborts whatever's scheduled).
    // Instead, handleReceived() below flips this flag off for just the
    // RANGE reception, and loop() flips it back on right after the CIR
    // capture -- letting the library's own startTransmit() do the actual
    // re-arm safely, exactly as it already does for every other message.
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

void computeRangeAsymmetric() {
    DW1000Time round1 = (timePollAckReceived - timePollSent).wrap();
    DW1000Time reply1 = (timePollAckSent - timePollReceived).wrap();
    DW1000Time round2 = (timeRangeReceived - timePollAckSent).wrap();
    DW1000Time reply2 = (timeRangeSent - timePollAckReceived).wrap();
    DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
    timeComputedRange.setTimestamp(tof);
}

// ---- accumulator clock force (verified against decadriver's _dwt_enableclocks) ----
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

// Captures CIR of whatever was JUST received (called right after RANGE
// reception, before we've re-enabled listening in any meaningful sense
// -- see design note at top of file). Prints one "CIR,..." line, raw
// registers + raw accumulator, same format cir_features.py expects.
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

    // RANGE line first (so a host script sees distance before the CIR payload)
    DebugSerial.print(F("RANGE,"));
    DebugSerial.print(cirSeq);       DebugSerial.print(',');
    DebugSerial.print(distance, 4);  DebugSerial.print(',');
    DebugSerial.print(rxPower, 4);   DebugSerial.print(',');
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

void loop() {
    int32_t curMillis = millis();
    if (!sentAck && !receivedAck) {
        if (curMillis - lastActivity > resetPeriod) {
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
        if (msgId != expectedMsgId) {
            protocolFailed = true;
        }
        if (msgId == POLL) {
            protocolFailed = false;
            DW1000.getReceiveTimestamp(timePollReceived);
            expectedMsgId = RANGE;
            transmitPollAck();
            noteActivity();
        } else if (msgId == RANGE) {
            DW1000.getReceiveTimestamp(timeRangeReceived);
            expectedMsgId = POLL;
            if (!protocolFailed) {
                timePollSent.setTimestamp(data + 1);
                timePollAckReceived.setTimestamp(data + 6);
                timeRangeSent.setTimestamp(data + 11);
                computeRangeAsymmetric();

                float distance = timeComputedRange.getAsMeters();

                // capture CIR of this RANGE reception -- the last message
                // of the round -- now that the range math is done, and
                // BEFORE restoring receivePermanently, so nothing
                // re-arms/invalidates the registers before we read them
                captureAndDumpCir(distance);

                // restore the persistent default -- transmitRangeReport()'s
                // own startTransmit() will now re-arm RX safely afterward,
                // exactly like stock (no manual newReceive()/idle() here)
                DW1000.receivePermanently(true);
                transmitRangeReport(timeComputedRange.getAsMicroSeconds());

                successRangingCount++;
                if (curMillis - rangingCountPeriod > 1000) {
                    samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
                    rangingCountPeriod = curMillis;
                    successRangingCount = 0;
                }
            } else {
                DW1000.receivePermanently(true);  // also needed on this path
                transmitRangeFailed();
            }
            noteActivity();
        }
    }
}
