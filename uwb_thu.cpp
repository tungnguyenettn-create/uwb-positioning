#include <Arduino.h>
#include <SPI.h>
#include <DW1000.h>
#include <DW1000Ranging.h>
#include <math.h>

// NodeMCU-BU01 theo pin.md:
// CSN=PA4, CLK=PA5, MISO=PA6, MOSI=PA7, IRQ=PB0, RST=PB12.
static constexpr uint8_t DW_CS  = PA4;
static constexpr uint8_t DW_IRQ = PB0;
static constexpr uint8_t DW_RST = PB12;

// Hai board phai dung EUI khac nhau.
static char ANCHOR_EUI[] = "82:17:5B:D5:A9:9A:E2:9C";

// Antenna delay la gia tri RIENG cho moi board. 16427 la diem bat dau khi
// firmware cu do DAI hon thuc te xap xi 40 cm va ca hai board dung cung gia tri.
// Neu firmware cu do NGAN hon thuc te, giam gia tri nay thay vi tang.
// Sau khi do LOS o khoang cach chuan, cap nhat TAG va ANCHOR theo cung ket qua.
static constexpr uint16_t ANTENNA_DELAY = 16427;

// Cau hinh chinh xac cho channel 5 / PRF 64 MHz.
static constexpr uint8_t UWB_CHANNEL = DW1000.CHANNEL_5;
static constexpr uint8_t UWB_PREAMBLE_CODE = DW1000.PREAMBLE_CODE_64MHZ_10;

// RX - FP lon thuong cho thay nang luong phan xa chiem uu the. Nguong 10 dB
// la quy tac chuan doan NLOS; mau nhu vay khong duoc dua vao bo loc khoang cach.
static constexpr float MAX_LOS_POWER_DELTA_DB = 10.0f;

// UART1 vat ly cua board: RX=U1RX/PA10, TX=U1TX/PA9.
HardwareSerial DebugSerial(PA10, PA9);

class RobustRangeFilter {
public:
    void push(float sample)
    {
        samples_[next_] = sample;
        next_ = (next_ + 1U) % WINDOW_SIZE;
        if (count_ < WINDOW_SIZE) {
            ++count_;
        }

        const float median = medianOfSamples();
        if (!initialized_) {
            filtered_ = median;
            initialized_ = true;
        } else {
            // EMA sau median: on dinh khi dung yen nhung van theo kip tag di cham.
            filtered_ += EMA_ALPHA * (median - filtered_);
        }
    }

    bool valid() const { return initialized_; }
    float value() const { return filtered_; }

private:
    static constexpr uint8_t WINDOW_SIZE = 5;
    static constexpr float EMA_ALPHA = 0.35f;

    float samples_[WINDOW_SIZE] = {};
    uint8_t count_ = 0;
    uint8_t next_ = 0;
    bool initialized_ = false;
    float filtered_ = NAN;

    float medianOfSamples() const
    {
        float sorted[WINDOW_SIZE];
        for (uint8_t i = 0; i < count_; ++i) {
            sorted[i] = samples_[i];
        }

        for (uint8_t i = 0; i < count_; ++i) {
            for (uint8_t j = i + 1; j < count_; ++j) {
                if (sorted[j] < sorted[i]) {
                    const float temporary = sorted[i];
                    sorted[i] = sorted[j];
                    sorted[j] = temporary;
                }
            }
        }

        if ((count_ & 1U) != 0U) {
            return sorted[count_ / 2U];
        }
        return 0.5f * (sorted[count_ / 2U - 1U] + sorted[count_ / 2U]);
    }
};

static RobustRangeFilter rangeFilter;
static uint32_t acceptedSamples = 0;
static uint32_t rejectedNlosSamples = 0;

static void onNewRange();
static void onNewBlink(DW1000Device *device);
static void onInactiveDevice(DW1000Device *device);
static void applyAccurateRadioSettings();
static void printRadioConfiguration();
static void printMeasurement(float filteredM, float rawM, float rxPowerDbm,
                             float fpPowerDbm, float quality, float pathDeltaDb);

void setup()
{
    DebugSerial.begin(115200);
    delay(1000);
    DebugSerial.println("UWB ANCHOR khoi dong");

    // SPI1 cua STM32F103: SCK=PA5, MISO=PA6, MOSI=PA7.
    DW1000Ranging.initCommunication(DW_RST, DW_CS, DW_IRQ);

    // Phai dat truoc startAsAnchor(): startAsAnchor() se commit gia tri nay vao
    // ca TX_ANTD va LDE_RXANTD cua DW1000.
    DW1000.setAntennaDelay(ANTENNA_DELAY);

    // Loc cua thu vien chay truoc callback va khong biet mau NLOS. Ta tat no,
    // sau do chi loc cac mau co dau hieu LOS trong onNewRange().
    DW1000Ranging.useRangeFilter(false);

    DW1000Ranging.attachNewRange(onNewRange);
    DW1000Ranging.attachBlinkDevice(onNewBlink);
    DW1000Ranging.attachInactiveDevice(onInactiveDevice);

    // Asymmetric double-sided TWR: 110 kbps, PRF 64 MHz, preamble 2048.
    DW1000Ranging.startAsAnchor(ANCHOR_EUI, DW1000.MODE_LONGDATA_RANGE_ACCURACY);

    // Thu vien 0.9 doi PRF sang 64 MHz nhung khong doi lai preamble code.
    // Commit lai channel sau khi start de ep code 10 dung cho CH5/PRF64,
    // sau do khoi dong lai receiver khi radio dang o trang thai idle.
    applyAccurateRadioSettings();
    printRadioConfiguration();
    DebugSerial.println("UWB ANCHOR dang cho TAG...");
}

void loop()
{
    DW1000Ranging.loop();
}

static void applyAccurateRadioSettings()
{
    DW1000.newConfiguration();
    DW1000.setChannel(UWB_CHANNEL);
    DW1000.setPreambleCode(UWB_PREAMBLE_CODE);
    DW1000.commitConfiguration();

    // newConfiguration() dua chip ve IDLE; phuc hoi receive mode cua ranging.
    DW1000.newReceive();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

static void printRadioConfiguration()
{
    char mode[96];
    DW1000.getPrintableDeviceMode(mode);
    DebugSerial.print("Radio: ");
    DebugSerial.println(mode);
    DebugSerial.print("Antenna delay: ");
    DebugSerial.println(DW1000.getAntennaDelay());
}

static void onNewRange()
{
    DW1000Device *device = DW1000Ranging.getDistantDevice();
    if (device == nullptr) {
        return;
    }

    const float rawM = device->getRange();
    const float rxPowerDbm = device->getRXPower();
    const float fpPowerDbm = device->getFPPower();
    const float quality = device->getQuality();
    const float pathDeltaDb = rxPowerDbm - fpPowerDbm;

    if (!isfinite(rawM) || !isfinite(rxPowerDbm) || !isfinite(fpPowerDbm)) {
        return;
    }

    if (pathDeltaDb > MAX_LOS_POWER_DELTA_DB) {
        ++rejectedNlosSamples;
        DebugSerial.print("BO QUA NLOS | raw: ");
        DebugSerial.print(rawM, 3);
        DebugSerial.print(" m | RX-FP: ");
        DebugSerial.print(pathDeltaDb, 1);
        DebugSerial.print(" dB | rejected: ");
        DebugSerial.println(rejectedNlosSamples);
        return;
    }

    rangeFilter.push(rawM);
    ++acceptedSamples;
    printMeasurement(rangeFilter.value(), rawM, rxPowerDbm, fpPowerDbm,
                     quality, pathDeltaDb);
}

static void printMeasurement(float filteredM, float rawM, float rxPowerDbm,
                             float fpPowerDbm, float quality, float pathDeltaDb)
{
    DebugSerial.print("Khoang cach: ");
    DebugSerial.print(filteredM, 3);
    DebugSerial.print(" m (");
    DebugSerial.print(filteredM * 100.0f, 1);
    DebugSerial.print(" cm) | raw: ");
    DebugSerial.print(rawM, 3);
    DebugSerial.print(" m | RX: ");
    DebugSerial.print(rxPowerDbm, 1);
    DebugSerial.print(" dBm | FP: ");
    DebugSerial.print(fpPowerDbm, 1);
    DebugSerial.print(" dBm | RX-FP: ");
    DebugSerial.print(pathDeltaDb, 1);
    DebugSerial.print(" dB | Q: ");
    DebugSerial.print(quality, 2);
    DebugSerial.print(" | ok: ");
    DebugSerial.println(acceptedSamples);
}

static void onNewBlink(DW1000Device *device)
{
    DebugSerial.print("Phat hien TAG, dia chi 0x");
    DebugSerial.println(device->getShortAddress(), HEX);
}

static void onInactiveDevice(DW1000Device *device)
{
    DebugSerial.print("Mat ket noi TAG, dia chi 0x");
    DebugSerial.println(device->getShortAddress(), HEX);
}
