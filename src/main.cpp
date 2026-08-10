#include <Arduino.h>
#include <SPI.h>
#include <DW1000.h>
#include <DW1000Ranging.h>

// NodeMCU-BU01 theo pin.md:
// CSN=PA4, CLK=PA5, MISO=PA6, MOSI=PA7, IRQ=PB0, RST=PB12.
static constexpr uint8_t DW_CS  = PA4;
static constexpr uint8_t DW_IRQ = PB0;
static constexpr uint8_t DW_RST = PB12;

// Hai board phai dung EUI khac nhau.
static char TAG_EUI[] = "7D:00:22:EA:82:60:3B:9C";

// Antenna delay la gia tri RIENG cho moi board. 16427 la diem bat dau khi
// firmware cu do DAI hon thuc te xap xi 40 cm va ca hai board dung cung gia tri.
// Neu firmware cu do NGAN hon thuc te, giam gia tri nay thay vi tang.
// Sau khi do LOS o khoang cach chuan, cap nhat TAG va ANCHOR theo cung ket qua.
static constexpr uint16_t ANTENNA_DELAY = 16427;

// Cau hinh chinh xac cho channel 5 / PRF 64 MHz.
static constexpr uint8_t UWB_CHANNEL = DW1000.CHANNEL_5;
static constexpr uint8_t UWB_PREAMBLE_CODE = DW1000.PREAMBLE_CODE_64MHZ_10;

// UART1 vat ly cua board: RX=U1RX/PA10, TX=U1TX/PA9.
HardwareSerial DebugSerial(PA10, PA9);

static void onNewRange();
static void onNewDevice(DW1000Device *device);
static void onInactiveDevice(DW1000Device *device);
static void applyAccurateRadioSettings();
static void printRadioConfiguration();

void setup()
{
    DebugSerial.begin(115200);
    delay(1000);
    DebugSerial.println("UWB TAG khoi dong");

    // SPI1 cua STM32F103: SCK=PA5, MISO=PA6, MOSI=PA7.
    DW1000Ranging.initCommunication(DW_RST, DW_CS, DW_IRQ);

    // Phai dat truoc startAsTag(): startAsTag() se commit gia tri nay vao
    // ca TX_ANTD va LDE_RXANTD cua DW1000.
    DW1000.setAntennaDelay(ANTENNA_DELAY);

    // Tu dung robust filter o ANCHOR sau khi da loai mau NLOS. Khong bat EMA
    // co san cua thu vien, vi no se lam tron ca mau phan xa/NLOS.
    DW1000Ranging.useRangeFilter(false);

    DW1000Ranging.attachNewRange(onNewRange);
    DW1000Ranging.attachNewDevice(onNewDevice);
    DW1000Ranging.attachInactiveDevice(onInactiveDevice);

    // Asymmetric double-sided TWR: 110 kbps, PRF 64 MHz, preamble 2048.
    DW1000Ranging.startAsTag(TAG_EUI, DW1000.MODE_LONGDATA_RANGE_ACCURACY);

    // Thu vien 0.9 doi PRF sang 64 MHz nhung khong doi lai preamble code.
    // Commit lai channel sau khi start de ep code 10 dung cho CH5/PRF64,
    // sau do khoi dong lai receiver khi radio dang o trang thai idle.
    applyAccurateRadioSettings();
    printRadioConfiguration();
    DebugSerial.println("UWB TAG dang tim ANCHOR...");
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
    // ANCHOR la noi in gia tri range va danh gia chat luong duong truyen.
}

static void onNewDevice(DW1000Device *device)
{
    DebugSerial.print("Da ket noi ANCHOR, dia chi 0x");
    DebugSerial.println(device->getShortAddress(), HEX);
}

static void onInactiveDevice(DW1000Device *device)
{
    DebugSerial.print("Mat ket noi ANCHOR, dia chi 0x");
    DebugSerial.println(device->getShortAddress(), HEX);
}
