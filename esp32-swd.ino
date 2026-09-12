#include <Arduino.h>

constexpr uint8_t SWCLK = 18;
constexpr uint8_t SWDIO = 19;
constexpr uint32_t SWD_HALF_US = 1;

static void swdioOutput() {
    pinMode(SWDIO, OUTPUT);
}

static void swdioInput() {
    pinMode(SWDIO, INPUT);
}

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
    // SWD data is sampled on the rising edge. Read while SWCLK is low,
    // then raise SWCLK to sample the value for this bit.
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
    for (int i = 0; i < count; ++i) {
        swdWriteBit((value >> i) & 1U); // SWD is LSB-first.
    }
}

static uint32_t swdReadBits(int count) {
    uint32_t value = 0;
    for (int i = 0; i < count; ++i) {
        if (swdReadBit()) {
            value |= (1UL << i); // LSB-first.
        }
    }
    return value;
}

static void lineReset() {
    swdioOutput();
    digitalWrite(SWDIO, HIGH);
    for (int i = 0; i < 52; ++i) {
        clockOnce();
    }
}

static void idleClocks(int count) {
    swdioOutput();
    digitalWrite(SWDIO, LOW);
    for (int i = 0; i < count; ++i) {
        clockOnce();
    }
}

static void switchToSwd() {
    // ARM ADIv5 JTAG-to-SWD activation sequence, sent LSB-first.
    lineReset();
    swdioOutput();
    swdWriteBits(0xE79E, 16);
    lineReset();
    idleClocks(4);
}

static uint8_t makeRequest(bool ap, bool read, uint8_t addr) {
    // Request bits: START, APnDP, RnW, A2, A3, PARITY, STOP, PARK.
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
    for (int i = 0; i < 32; ++i) {
        parity ^= (value >> i) & 1U;
    }
    return parity == receivedParity;
}

static bool readDpIdcode(uint32_t &idcode) {
    constexpr uint8_t DP_IDCODE_REQUEST = 0xA5; // AP=0, READ=1, A[3:2]=00.

    swdioOutput();
    swdWriteBits(DP_IDCODE_REQUEST, 8);

    // Host-to-target turnaround: host releases SWDIO for one clock.
    swdioInput();
    clockOnce();

    uint8_t ack = static_cast<uint8_t>(swdReadBits(3));
    Serial.printf("ACK: 0b%03u", ack);
    if (ack == 0b001) {
        Serial.println(" (OK)");
    } else if (ack == 0b010) {
        Serial.println(" (WAIT)");
    } else if (ack == 0b100) {
        Serial.println(" (FAULT)");
    } else {
        Serial.println(" (INVALID)");
    }

    if (ack != 0b001) {
        swdioOutput();
        digitalWrite(SWDIO, LOW);
        clockOnce();
        return false;
    }

    idcode = swdReadBits(32);
    bool receivedParity = swdReadBit();
    bool parityOk = checkParity32(idcode, receivedParity);

    // Target-to-host turnaround: let the target finish driving the line
    // during the turnaround clock, then reclaim SWDIO.
    clockOnce();
    swdioOutput();
    digitalWrite(SWDIO, LOW);

    if (!parityOk) {
        Serial.println("IDCODE parity: ERROR");
        return false;
    }

    Serial.println("IDCODE parity: OK");
    return true;
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

    uint32_t idcode = 0;
    if (readDpIdcode(idcode)) {
        Serial.printf("DP IDCODE = 0x%08lX\n", (unsigned long)idcode);
    } else {
        Serial.println("Failed to read DP IDCODE");
    }
}

void loop() {
    delay(1000);
}

