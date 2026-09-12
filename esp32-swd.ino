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

static bool memRead32(uint32_t address, uint32_t &value) {
    uint8_t ack = 0;
    if (!swdWriteAP(0x04, address, ack)) return false;
    return swdReadAP(0x0C, value);
}

static bool memWrite32(uint32_t address, uint32_t value) {
    uint8_t ack = 0;
    if (!swdWriteAP(0x04, address, ack)) return false;
    return swdWriteAP(0x0C, value, ack);
}

static bool memWrite16(uint32_t address, uint16_t value) {
    uint8_t ack = 0;
    if (!swdWriteAP(0x04, address, ack)) return false;
    if (!swdWriteAP(0x00, 0x23000051UL, ack)) return false;
    bool ok = swdWriteAP(0x0C, static_cast<uint32_t>(value), ack);
    if (!swdWriteAP(0x00, 0x23000052UL, ack)) return false;
    return ok;
}

static bool flashProgramHalfword(uint32_t address, uint16_t value) {
    constexpr uint32_t FLASH_SR_ADDR = 0x4002200CUL;
    constexpr uint32_t FLASH_CR_ADDR = 0x40022010UL;
    constexpr uint32_t FLASH_SR_BSY = 0x00000001UL;
    constexpr uint32_t FLASH_SR_PGERR = 0x00000004UL;
    constexpr uint32_t FLASH_SR_WRPRTERR = 0x00000010UL;
    constexpr uint32_t FLASH_CR_PG = 0x00000001UL;
    constexpr uint32_t FLASH_TIMEOUT_MS = 5000UL;

    uint32_t sr = 0;
    uint32_t cr = 0;
    if (!memRead32(FLASH_SR_ADDR, sr)) return false;
    if ((sr & FLASH_SR_BSY) != 0) return false;
    if (!memRead32(FLASH_CR_ADDR, cr)) return false;
    if ((cr & 0x00000080UL) != 0) return false;

    cr |= FLASH_CR_PG;
    if (!memWrite32(FLASH_CR_ADDR, cr)) return false;

    uint32_t crConfirm = 0;
    if (!memRead32(FLASH_CR_ADDR, crConfirm)) return false;
    if ((crConfirm & FLASH_CR_PG) == 0 || (crConfirm & 0x00000080UL) != 0) return false;

    if (!memWrite16(address, value)) return false;

    const unsigned long startMs = millis();
    while (true) {
        if (!memRead32(FLASH_SR_ADDR, sr)) return false;
        if ((sr & FLASH_SR_BSY) == 0) break;
        if ((millis() - startMs) >= FLASH_TIMEOUT_MS) return false;
        delay(1);
    }

    if ((sr & FLASH_SR_PGERR) != 0 || (sr & FLASH_SR_WRPRTERR) != 0) return false;

    cr = crConfirm & ~FLASH_CR_PG;
    return memWrite32(FLASH_CR_ADDR, cr);
}

static bool cortexReadDHCSR(uint32_t &value) {
    return memRead32(0xE000EDF0, value);
}

static bool cortexHalt() {
    return memWrite32(0xE000EDF0, 0xA05F0003);
}

static bool cortexResume() {
    return memWrite32(0xE000EDF0, 0xA05F0001);
}

static void printDhcsr(uint32_t value, const char *label) {
    Serial.printf("%s\n", label);
    Serial.printf("0x%08lX\n", (unsigned long)value);
    Serial.printf("C_DEBUGEN = %u\n", (unsigned)((value >> 0) & 1U));
    Serial.printf("C_HALT    = %u\n", (unsigned)((value >> 1) & 1U));
}

static bool selectApBank0() {
    uint8_t ack = 0;
    return swdWriteDP(0x08, 0x00000000, ack);
}

static bool setupMemAp32() {
    uint8_t ack = 0;
    if (!selectApBank0()) return false;
    return swdWriteAP(0x00, 0x23000052, ack);
}
static bool flashErasePage(uint32_t pageAddress) {
    constexpr uint32_t FLASH_SR_ADDR = 0x4002200CUL;
    constexpr uint32_t FLASH_CR_ADDR = 0x40022010UL;
    constexpr uint32_t FLASH_AR_ADDR = 0x40022014UL;
    constexpr uint32_t FLASH_SR_BSY = 0x00000001UL;
    constexpr uint32_t FLASH_SR_PGERR = 0x00000004UL;
    constexpr uint32_t FLASH_SR_WRPRTERR = 0x00000010UL;
    constexpr uint32_t FLASH_CR_PER = 0x00000002UL;
    constexpr uint32_t FLASH_CR_STRT = 0x00000040UL;
    constexpr uint32_t FLASH_CR_LOCK = 0x00000080UL;
    constexpr uint32_t FLASH_TIMEOUT_MS = 5000UL;

    Serial.println();
    Serial.println("[13] STM32F1 FLASH SINGLE PAGE ERASE");
    uint32_t sr = 0;
    uint32_t cr = 0;
    if (!memRead32(FLASH_SR_ADDR, sr)) {
        Serial.println("FLASH SR BEFORE: READ FAILED");
        return false;
    }
    Serial.printf("ERASE ADDRESS       = 0x%08lX\n", (unsigned long)pageAddress);
    Serial.printf("FLASH SR BEFORE     = 0x%08lX\n", (unsigned long)sr);
    if ((sr & FLASH_SR_BSY) != 0) {
        Serial.println("FLASH PAGE ERASE: FAILED (FLASH BUSY)");
        return false;
    }
    if (!memRead32(FLASH_CR_ADDR, cr)) {
        Serial.println("FLASH CR BEFORE: READ FAILED");
        return false;
    }
    Serial.printf("FLASH CR BEFORE     = 0x%08lX\n", (unsigned long)cr);
    if ((cr & FLASH_CR_LOCK) != 0) {
        Serial.println("FLASH PAGE ERASE: FAILED (FLASH LOCKED)");
        return false;
    }
    cr |= FLASH_CR_PER;
    if (!memWrite32(FLASH_CR_ADDR, cr)) {
        Serial.println("FLASH CR PER WRITE: FAILED");
        return false;
    }
    Serial.println("FLASH PER           = 1");
    uint32_t crConfirm = 0;
    if (!memRead32(FLASH_CR_ADDR, crConfirm)) {
        Serial.println("FLASH CR PER CONFIRM: READ FAILED");
        return false;
    }
    if ((crConfirm & FLASH_CR_LOCK) != 0 || (crConfirm & FLASH_CR_PER) == 0) {
        Serial.printf("FLASH CR PER CONFIRM = 0x%08lX\n", (unsigned long)crConfirm);
        Serial.println("FLASH PAGE ERASE: FAILED (PER/LOCK VERIFY)");
        return false;
    }
    if (!memWrite32(FLASH_AR_ADDR, pageAddress)) {
        Serial.println("FLASH AR WRITE: FAILED");
        return false;
    }
    Serial.printf("FLASH AR            = 0x%08lX\n", (unsigned long)pageAddress);
    cr = crConfirm | FLASH_CR_STRT;
    if (!memWrite32(FLASH_CR_ADDR, cr)) {
        Serial.println("FLASH START WRITE: FAILED");
        return false;
    }
    Serial.println("FLASH START         = 1");
    Serial.println("WAITING FOR BSY...");
    const unsigned long startMs = millis();
    while (true) {
        if (!memRead32(FLASH_SR_ADDR, sr)) {
            Serial.println("FLASH SR DURING ERASE: READ FAILED");
            return false;
        }
        if ((sr & FLASH_SR_BSY) == 0) break;
        if ((millis() - startMs) >= FLASH_TIMEOUT_MS) {
            Serial.println("FLASH PAGE ERASE: TIMEOUT");
            return false;
        }
        delay(1);
    }
    Serial.printf("FLASH SR AFTER      = 0x%08lX\n", (unsigned long)sr);
    if ((sr & FLASH_SR_PGERR) != 0 || (sr & FLASH_SR_WRPRTERR) != 0) {
        Serial.printf("FLASH ERROR FLAGS   = PGERR=%u WRPRTERR=%u\n",
                      (unsigned)((sr & FLASH_SR_PGERR) != 0),
                      (unsigned)((sr & FLASH_SR_WRPRTERR) != 0));
        cr = crConfirm & ~FLASH_CR_PER;
        if (!memWrite32(FLASH_CR_ADDR, cr)) Serial.println("FLASH PER CLEAR: WRITE FAILED");
        Serial.println("FLASH PAGE ERASE: FAIL");
        return false;
    }
    cr = crConfirm & ~FLASH_CR_PER;
    if (!memWrite32(FLASH_CR_ADDR, cr)) {
        Serial.println("FLASH PER CLEAR: WRITE FAILED");
        return false;
    }
    Serial.println("FLASH PER           = 0");
    uint32_t crAfter = 0;
    if (!memRead32(FLASH_CR_ADDR, crAfter)) {
        Serial.println("FLASH CR AFTER: READ FAILED");
        return false;
    }
    if ((crAfter & FLASH_CR_LOCK) != 0 || (crAfter & FLASH_CR_PER) != 0) {
        Serial.printf("FLASH CR AFTER      = 0x%08lX\n", (unsigned long)crAfter);
        Serial.println("FLASH PAGE ERASE: FAIL (PER/LOCK FINAL VERIFY)");
        return false;
    }
    Serial.println("FLASH PAGE ERASE: OK");
    return true;
}

static bool flashVerifyErasePage0() {
    constexpr uint32_t FLASH_BASE = 0x08000000UL;
    constexpr uint32_t EXPECTED = 0xFFFFFFFFUL;

    Serial.println();
    Serial.println("[14] STM32F1 FLASH ERASE VERIFY");

    for (uint32_t offset = 0; offset < 16UL; offset += 4UL) {
        uint32_t address = FLASH_BASE + offset;
        uint32_t value = 0;
        if (!memRead32(address, value)) {
            Serial.printf("ADDRESS 0x%08lX = READ FAILED\n", (unsigned long)address);
            Serial.println("FLASH ERASE VERIFY: FAIL");
            return false;
        }

        Serial.printf("ADDRESS 0x%08lX = 0x%08lX\n",
                      (unsigned long)address,
                      (unsigned long)value);

        if (value != EXPECTED) {
            Serial.printf("FLASH ERASE VERIFY: FAIL (0x%08lX != 0xFFFFFFFF)\n",
                          (unsigned long)value);
            return false;
        }
    }

    Serial.println("FLASH ERASE VERIFY: OK");
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

    Serial.println();
    Serial.println("[8] STM32F1 FLASH CONTROLLER REGISTERS (READ ONLY)");
    if (!setupMemAp32()) {
        Serial.println("      MEM-AP SETUP FAILED");
        return;
    }

    struct FlashReg { uint32_t address; const char *name; };
    const FlashReg regs[] = {
        {0x40022000, "FLASH_ACR"},
        {0x40022004, "FLASH_KEYR"},
        {0x40022008, "FLASH_OPTKEYR"},
        {0x4002200C, "FLASH_SR"},
        {0x40022010, "FLASH_CR"},
        {0x40022014, "FLASH_AR"},
        {0x4002201C, "FLASH_OBR"},
        {0x40022020, "FLASH_WRPR"},
    };

    bool flashRegsOk = true;
    for (const auto &reg : regs) {
        if (!memRead32(reg.address, value)) {
            Serial.printf("      %s 0x%08lX = READ FAILED\n", reg.name, (unsigned long)reg.address);
            flashRegsOk = false;
            break;
        }
        Serial.printf("      %-13s 0x%08lX = 0x%08lX\n", reg.name, (unsigned long)reg.address, (unsigned long)value);
    }

    Serial.println();
    Serial.printf("FLASH REGISTER READ: %s\n", flashRegsOk ? "OK" : "FAILED");
    if (!flashRegsOk) return;

    Serial.println();
    Serial.println("[9] CORTEX-M3 DEBUG");

    if (!cortexReadDHCSR(value)) {
        Serial.println("DHCSR READ BEFORE: FAILED");
        return;
    }
    printDhcsr(value, "DHCSR BEFORE:");

    if (!cortexHalt()) {
        Serial.println("HALT:");
        Serial.println("WRITE ACK = FAILED");
        return;
    }
    Serial.println("HALT:");
    Serial.println("WRITE ACK = OK");

    uint32_t dhcsrHalt = 0;
    if (!cortexReadDHCSR(dhcsrHalt)) {
        Serial.println("DHCSR AFTER HALT: READ FAILED");
        return;
    }
    printDhcsr(dhcsrHalt, "DHCSR AFTER HALT:");
    if ((dhcsrHalt & 0x00000003UL) != 0x00000003UL) {
        Serial.println("CORTEX DEBUG TEST: FAILED (HALT VERIFY)");
        return;
    }

    if (!cortexResume()) {
        Serial.println("RESUME:");
        Serial.println("WRITE ACK = FAILED");
        return;
    }
    Serial.println("RESUME:");
    Serial.println("WRITE ACK = OK");

    uint32_t dhcsrRun = 0;
    if (!cortexReadDHCSR(dhcsrRun)) {
        Serial.println("DHCSR AFTER RESUME: READ FAILED");
        return;
    }
    printDhcsr(dhcsrRun, "DHCSR AFTER RESUME:");
    if ((dhcsrRun & 0x00000003UL) != 0x00000001UL) {
        Serial.println("CORTEX DEBUG TEST: FAILED (RESUME VERIFY)");
        return;
    }

    Serial.println("CORTEX DEBUG TEST: OK");

    Serial.println();
    Serial.println("[10] STM32F1 FLASH SIZE DETECTION (READ ONLY)");
    uint32_t flashSizeInfo = 0;
    if (!memRead32(0x1FFFF7E0, flashSizeInfo)) {
        Serial.println("FLASH SIZE REGISTER READ: FAILED");
        return;
    }

    uint32_t flashSizeKB = flashSizeInfo & 0xFFFFUL;
    Serial.printf("FLASH SIZE REG 0x1FFFF7E0 = 0x%08lX\n", (unsigned long)flashSizeInfo);
    Serial.printf("FLASH SIZE        = %lu KB\n", (unsigned long)flashSizeKB);

    if (flashSizeKB == 0 || flashSizeKB == 0xFFFFUL) {
        Serial.println("FLASH SIZE DETECTION: FAILED (INVALID FACTORY VALUE)");
        return;
    }

    Serial.printf("FLASH RANGE       = 0x08000000 - 0x%08lX\n",
                  (unsigned long)(0x08000000UL + flashSizeKB * 1024UL - 1UL));
    Serial.println("FLASH SIZE DETECTION: OK");
    Serial.println();
    Serial.println("[11] STM32F1 FLASH PAGE/RANGE DETECTION (READ ONLY)");

    uint32_t flashBase = 0x08000000UL;
    uint32_t flashEnd = flashBase + flashSizeKB * 1024UL - 1UL;
    uint32_t pageSize = 0;
    const char *densityName = "UNKNOWN";

    // STM32F103 medium-density devices use 1 KB pages.
    if (flashSizeKB > 0 && flashSizeKB <= 128) {
        pageSize = 1024UL;
        densityName = "MEDIUM DENSITY";
    }

    if (pageSize == 0) {
        Serial.println("FLASH PAGE/RANGE DETECTION: FAILED (UNSUPPORTED DENSITY)");
        return;
    }

    if (((flashEnd - flashBase + 1UL) % pageSize) != 0) {
        Serial.println("FLASH PAGE/RANGE DETECTION: FAILED (RANGE NOT PAGE ALIGNED)");
        return;
    }

    uint32_t pageCount = flashSizeKB * 1024UL / pageSize;

    Serial.printf("FLASH BASE        = 0x%08lX\n", (unsigned long)flashBase);
    Serial.printf("FLASH SIZE        = %lu KB\n", (unsigned long)flashSizeKB);
    Serial.printf("FLASH END         = 0x%08lX\n", (unsigned long)flashEnd);
    Serial.printf("FLASH DENSITY     = %s\n", densityName);
    Serial.printf("PAGE SIZE         = %lu bytes\n", (unsigned long)pageSize);
    Serial.printf("PAGE COUNT        = %lu\n", (unsigned long)pageCount);
    Serial.println("FLASH PAGE/RANGE DETECTION: OK");
    Serial.println();
    Serial.println("[12] STM32F1 FLASH UNLOCK");

    uint32_t flashSrBeforeUnlock = 0;
    if (!memRead32(0x4002200C, flashSrBeforeUnlock)) {
        Serial.println("FLASH SR BEFORE UNLOCK: READ FAILED");
        return;
    }
    Serial.printf("FLASH SR BEFORE UNLOCK = 0x%08lX\n", (unsigned long)flashSrBeforeUnlock);
    if ((flashSrBeforeUnlock & 0x00000001UL) != 0) {
        Serial.println("FLASH UNLOCK: FAILED (FLASH BUSY)");
        return;
    }

    uint32_t flashCrBeforeUnlock = 0;
    if (!memRead32(0x40022010, flashCrBeforeUnlock)) {
        Serial.println("FLASH CR BEFORE UNLOCK: READ FAILED");
        return;
    }
    Serial.printf("FLASH CR BEFORE UNLOCK = 0x%08lX\n", (unsigned long)flashCrBeforeUnlock);

    const uint32_t FLASH_CR_LOCK = 0x00000080UL;
    if ((flashCrBeforeUnlock & FLASH_CR_LOCK) == 0) {
        Serial.println("FLASH ALREADY UNLOCKED");
        uint32_t flashCrConfirm = 0;
        if (!memRead32(0x40022010, flashCrConfirm)) {
            Serial.println("FLASH LOCK CONFIRM: READ FAILED");
            return;
        }
        Serial.printf("FLASH CR CONFIRM       = 0x%08lX\n", (unsigned long)flashCrConfirm);
        if ((flashCrConfirm & FLASH_CR_LOCK) != 0) {
            Serial.println("FLASH UNLOCK: FAILED (LOCK STILL SET)");
            return;
        }
        Serial.println("FLASH UNLOCK: OK");
        if (!flashErasePage(0x08000000UL)) return;
        if (!flashVerifyErasePage0()) return;
        return;
    }

    Serial.println("FLASH LOCK = 1");
    Serial.println("WRITING FLASH KEYR #1");
    if (!memWrite32(0x40022004, 0x45670123UL)) {
        Serial.println("FLASH KEYR #1: WRITE FAILED");
        return;
    }
    Serial.println("FLASH KEYR #1: WRITE ACK OK");

    Serial.println("WRITING FLASH KEYR #2");
    if (!memWrite32(0x40022004, 0xCDEF89ABUL)) {
        Serial.println("FLASH KEYR #2: WRITE FAILED");
        return;
    }
    Serial.println("FLASH KEYR #2: WRITE ACK OK");

    uint32_t flashCrAfterUnlock = 0;
    if (!memRead32(0x40022010, flashCrAfterUnlock)) {
        Serial.println("FLASH CR AFTER UNLOCK: READ FAILED");
        return;
    }
    Serial.printf("FLASH CR AFTER UNLOCK  = 0x%08lX\n", (unsigned long)flashCrAfterUnlock);
    if ((flashCrAfterUnlock & FLASH_CR_LOCK) != 0) {
        Serial.println("FLASH UNLOCK: FAILED (LOCK STILL SET)");
        return;
    }
    Serial.println("FLASH LOCK = 0");
    Serial.println("FLASH UNLOCK: OK");

    if (!flashErasePage(0x08000000UL)) return;
    if (!flashVerifyErasePage0()) return;
}

void loop() { delay(1000); }
