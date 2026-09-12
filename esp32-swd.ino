#include <Arduino.h>

constexpr uint8_t SWCLK = 18;
constexpr uint8_t SWDIO = 19;
constexpr uint32_t SWD_HALF_US = 1;

static void swdioOutput() { pinMode(SWDIO, OUTPUT); }
static void swdioInput() { pinMode(SWDIO, INPUT); }

static void clockOnce() {
    digitalWrite(SWCLK, HIGH);
    delayMicroseconds(SWD_HALF_US);
    digitalWrite(SWCLK, LOW);
    delayMicroseconds(SWD_HALF_US);
}

static void swdWriteBit(bool bit) {
    digitalWrite(SWDIO, bit ? HIGH : LOW);
    clockOnce();
}

static bool swdReadBit() {
    digitalWrite(SWCLK, LOW);
    delayMicroseconds(SWD_HALF_US);
    bool bit = digitalRead(SWDIO);
    digitalWrite(SWCLK, HIGH);
    delayMicroseconds(SWD_HALF_US);
    digitalWrite(SWCLK, LOW);
    delayMicroseconds(SWD_HALF_US);
    return bit;
}

static void swdWriteBits(uint32_t value, int count) {
    for (int i = 0; i < count; ++i) swdWriteBit((value >> i) & 1U);
}

static uint32_t swdReadBits(int count) {
    uint32_t value = 0;
    for (int i = 0; i < count; ++i) {
        if (swdReadBit()) value |= (1UL << i);
    }
    return value;
}

static void lineReset() {
    swdioOutput();
    digitalWrite(SWDIO, HIGH);
    for (int i = 0; i < 52; ++i) clockOnce();
}

static void idleClocks(int count) {
    swdioOutput();
    digitalWrite(SWDIO, LOW);
    for (int i = 0; i < count; ++i) clockOnce();
}

static void switchToSwd() {
    lineReset();
    swdioOutput();
    swdWriteBits(0xE79E, 16);
    lineReset();
    idleClocks(4);
}

static uint8_t makeRequest(bool ap, bool read, uint8_t addr) {
    uint8_t a2 = (addr >> 2) & 1U;
    uint8_t a3 = (addr >> 3) & 1U;
    uint8_t parity = (ap ? 1U : 0U) ^ (read ? 1U : 0U) ^ a2 ^ a3;
    return 0x01U |
           ((ap ? 1U : 0U) << 1) |
           ((read ? 1U : 0U) << 2) |
           (a2 << 3) |
           (a3 << 4) |
           (parity << 5) |
           (0U << 6) |
           (1U << 7);
}

static bool checkParity32(uint32_t value, bool receivedParity) {
    bool parity = false;
    for (int i = 0; i < 32; ++i) parity ^= (value >> i) & 1U;
    return parity == receivedParity;
}

static const char *ackName(uint8_t ack) {
    switch (ack) {
        case 0b001: return "OK";
        case 0b010: return "WAIT";
        case 0b100: return "FAULT";
        default: return "INVALID";
    }
}

static bool swdReadDP(uint8_t address, uint32_t &value, uint8_t &ackOut, bool &parityOkOut) {
    swdioOutput();
    swdWriteBits(makeRequest(false, true, address), 8);
    swdioInput();
    clockOnce();

    uint8_t ack = static_cast<uint8_t>(swdReadBits(3));
    ackOut = ack;
    parityOkOut = false;
    if (ack != 0b001) {
        swdioOutput();
        digitalWrite(SWDIO, LOW);
        clockOnce();
        return false;
    }

    value = swdReadBits(32);
    bool receivedParity = swdReadBit();
    bool parityOk = checkParity32(value, receivedParity);
    parityOkOut = parityOk;

    clockOnce();
    swdioOutput();
    digitalWrite(SWDIO, LOW);
    return parityOk;
}

static bool swdReadDP(uint8_t address, uint32_t &value) {
    uint8_t ack = 0;
    bool parityOk = false;
    bool ok = swdReadDP(address, value, ack, parityOk);
    Serial.printf("      ACK: 0b%03u (%s)\n", ack, ackName(ack));
    Serial.printf("      DATA: 0x%08lX\n", (unsigned long)value);
    Serial.printf("      PARITY: %s\n", parityOk ? "OK" : "ERROR");
    return ok;
}

static bool swdWriteDP(uint8_t address, uint32_t value, uint8_t &ackOut) {
    swdioOutput();
    swdWriteBits(makeRequest(false, false, address), 8);
    swdioInput();
    clockOnce();

    uint8_t ack = static_cast<uint8_t>(swdReadBits(3));
    ackOut = ack;
    if (ack != 0b001) {
        swdioOutput();
        digitalWrite(SWDIO, LOW);
        clockOnce();
        return false;
    }

    // Target-to-host turnaround before the host drives write data.
    swdioInput();
    clockOnce();
    swdioOutput();
    bool parity = false;
    for (int i = 0; i < 32; ++i) {
        bool bit = (value >> i) & 1U;
        parity ^= bit;
        swdWriteBit(bit);
    }
    swdWriteBit(parity);
    // STM32F1 SW-DP needs extra SWCLK cycles after a write for the
    // asynchronous SWCLK/HCLK write to become effective.
    digitalWrite(SWDIO, LOW);
    clockOnce();
    clockOnce();
    return true;
}

static bool swdWriteAP(uint8_t address, uint32_t value, uint8_t &ackOut) {
    swdioOutput();
    swdWriteBits(makeRequest(true, false, address), 8);
    swdioInput();
    clockOnce();

    uint8_t ack = static_cast<uint8_t>(swdReadBits(3));
    ackOut = ack;
    if (ack != 0b001) {
        swdioOutput();
        digitalWrite(SWDIO, LOW);
        clockOnce();
        return false;
    }

    swdioInput();
    clockOnce();
    swdioOutput();
    bool parity = false;
    for (int i = 0; i < 32; ++i) {
        bool bit = (value >> i) & 1U;
        parity ^= bit;
        swdWriteBit(bit);
    }
    swdWriteBit(parity);
    digitalWrite(SWDIO, LOW);
    clockOnce();
    clockOnce();
    return true;
}

static bool swdReadAP(uint8_t address, uint32_t &value) {
    // AP reads are posted. Issue the AP read, then fetch its result via DP RDBUFF.
    swdioOutput();
    swdWriteBits(makeRequest(true, true, address), 8);
    swdioInput();
    clockOnce();

    uint8_t ack = static_cast<uint8_t>(swdReadBits(3));
    Serial.printf("      AP ACK: 0b%03u (%s)\n", ack, ackName(ack));
    if (ack != 0b001) {
        swdioOutput();
        digitalWrite(SWDIO, LOW);
        clockOnce();
        return false;
    }

    // Consume the AP transaction's data phase; the posted result is read from RDBUFF.
    uint32_t ignored = swdReadBits(32);
    bool receivedParity = swdReadBit();
    bool parityOk = checkParity32(ignored, receivedParity);
    Serial.printf("      AP DATA PARITY: %s\n", parityOk ? "OK" : "ERROR");
    clockOnce();
    swdioOutput();
    digitalWrite(SWDIO, LOW);
    if (!parityOk) return false;

    return swdReadDP(0x0C, value);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    pinMode(SWCLK, OUTPUT);
    digitalWrite(SWCLK, LOW);
    swdioOutput();
    digitalWrite(SWDIO, LOW);

    Serial.println();
    Serial.println("ESP32 SWD programmer");
    Serial.println("--------------------");
    Serial.println("SWCLK = GPIO18, SWDIO = GPIO19");

    switchToSwd();

    uint32_t value = 0;
    uint8_t writeAck = 0;

    Serial.println();
    Serial.println("[1] DP IDCODE");
    if (!swdReadDP(0x00, value)) { Serial.println("      FAILED"); return; }
    Serial.printf("      IDCODE: 0x%08lX\n", (unsigned long)value);

    Serial.println();
    Serial.println("[2] DP ABORT/CLEAR ERRORS");
    if (!swdWriteDP(0x00, 0x0000001E, writeAck)) {
        Serial.printf("      ACK: 0b%03u (%s)\n", writeAck, ackName(writeAck));
        Serial.println("      FAILED");
        return;
    }
    Serial.printf("      ACK: 0b%03u (%s)\n", writeAck, ackName(writeAck));

    Serial.println();
    Serial.println("[3] DP CTRL/STAT");
    if (!swdReadDP(0x04, value)) { Serial.println("      FAILED"); return; }

    Serial.println();
    Serial.println("[4] DP POWER-UP");
    if (!swdWriteDP(0x04, 0x50000000, writeAck)) {
        Serial.printf("      ACK: 0b%03u (%s)\n", writeAck, ackName(writeAck));
        Serial.println("      FAILED");
        return;
    }
    Serial.printf("      WRITE ACK: 0b%03u (%s)\n", writeAck, ackName(writeAck));
    delayMicroseconds(10);
    if (!swdReadDP(0x04, value)) { Serial.println("      POWER-UP STATUS FAILED"); return; }

    Serial.println();
    Serial.println("[5] DP SELECT -> APBANKSEL=0xF");
    uint8_t selectAck = 0;
    if (!swdWriteDP(0x08, 0x000000F0, selectAck)) {
        Serial.printf("      ACK: 0b%03u (%s)\n", selectAck, ackName(selectAck));
        Serial.println("      FAILED");
        return;
    }
    Serial.printf("      ACK: 0b%03u (%s)\n", selectAck, ackName(selectAck));

    Serial.println();
    Serial.println("[6] AP IDR");
    if (!swdReadAP(0xFC, value)) { Serial.println("      FAILED"); return; }
    Serial.printf("      AP IDR: 0x%08lX\n", (unsigned long)value);

    Serial.println();
    Serial.println("[7] MEM-AP FLASH READ");
    if (!swdWriteDP(0x08, 0x00000000, selectAck)) {
        Serial.printf("      SELECT BANK0 ACK: 0b%03u (%s)\n", selectAck, ackName(selectAck));
        Serial.println("      FAILED");
        return;
    }
    Serial.printf("      SELECT BANK0 ACK: 0b%03u (%s)\n", selectAck, ackName(selectAck));

    uint8_t apAck = 0;
    if (!swdWriteAP(0x00, 0x23000052, apAck)) {
        Serial.printf("      CSW WRITE ACK: 0b%03u (%s)\n", apAck, ackName(apAck));
        Serial.println("      FAILED");
        return;
    }
    Serial.printf("      CSW WRITE ACK: 0b%03u (%s)\n", apAck, ackName(apAck));

    if (!swdWriteAP(0x04, 0x08000000, apAck)) {
        Serial.printf("      TAR WRITE ACK: 0b%03u (%s)\n", apAck, ackName(apAck));
        Serial.println("      FAILED");
        return;
    }
    Serial.printf("      TAR WRITE ACK: 0b%03u (%s)\n", apAck, ackName(apAck));

    for (int i = 0; i < 4; ++i) {
        if (!swdReadAP(0x0C, value)) {
            Serial.printf("      READ[%d] FAILED\n", i);
            return;
        }
        Serial.printf("      [0x%08lX] = 0x%08lX\n", 0x08000000UL + (unsigned long)(i * 4), (unsigned long)value);
    }
}

void loop() { delay(1000); }
