#include <Arduino.h>

constexpr uint8_t SWCLK = 18, SWDIO = 19;
constexpr uint32_t HALF_US = 1;
constexpr uint32_t FLASH_BASE = 0x08000000 UL, FLASH_SIZE_REG = 0x1FFFF7E0 UL;
constexpr uint32_t FLASH_SR = 0x4002200C UL, FLASH_CR = 0x40022010 UL, FLASH_AR = 0x40022014 UL, FLASH_KEYR = 0x40022004 UL;
constexpr uint32_t PAGE_SIZE = 1024 UL;

static void out() {
  pinMode(SWDIO, OUTPUT);
}
static void in () {
  pinMode(SWDIO, INPUT);
}
static void clk() {
  digitalWrite(SWCLK, HIGH);
  delayMicroseconds(HALF_US);
  digitalWrite(SWCLK, LOW);
  delayMicroseconds(HALF_US);
}
static void wb(bool b) {
  digitalWrite(SWDIO, b ? HIGH : LOW);
  clk();
}
static bool rb() {
  digitalWrite(SWCLK, LOW);
  delayMicroseconds(HALF_US);
  bool b = digitalRead(SWDIO);
  digitalWrite(SWCLK, HIGH);
  delayMicroseconds(HALF_US);
  digitalWrite(SWCLK, LOW);
  delayMicroseconds(HALF_US);
  return b;
}
static void wbits(uint32_t v, int n) {
  for (int i = 0; i < n; i++) wb((v >> i) & 1 U);
}
static uint32_t rbits(int n) {
  uint32_t v = 0;
  for (int i = 0; i < n; i++)
    if (rb()) v |= 1 UL << i;
  return v;
}
static void resetLine() {
  out();
  digitalWrite(SWDIO, HIGH);
  for (int i = 0; i < 60; i++) clk();
}
static void swdInit() {
  resetLine();
  out();
  wbits(0xE79E, 16);
  resetLine();
  out();
  digitalWrite(SWDIO, LOW);
  for (int i = 0; i < 4; i++) clk();
}
static uint8_t req(bool ap, bool rd, uint8_t a) {
  uint8_t a2 = (a >> 2) & 1 U, a3 = (a >> 3) & 1 U, p = (ap ? 1 : 0) ^ (rd ? 1 : 0) ^ a2 ^ a3;
  return 0x81 U | (ap << 1) | (rd << 2) | (a2 << 3) | (a3 << 4) | (p << 5);
}
static bool par(uint32_t v, bool p) {
  bool x = 0;
  for (int i = 0; i < 32; i++) x ^= (v >> i) & 1 U;
  return x == p;
}
static bool dpRead(uint8_t a, uint32_t & v) {
  out();
  wbits(req(false, true, a), 8);
  in();
  clk();
  uint8_t ack = rbits(3);
  if (ack != 1) {
    out();
    digitalWrite(SWDIO, LOW);
    clk();
    return false;
  }
  v = rbits(32);
  bool p = rb();
  clk();
  out();
  digitalWrite(SWDIO, LOW);
  return par(v, p);
}
static bool dpWrite(uint8_t a, uint32_t v) {
  out();
  wbits(req(false, false, a), 8);
  in();
  clk();
  uint8_t ack = rbits(3);
  if (ack != 1) {
    out();
    digitalWrite(SWDIO, LOW);
    clk();
    return false;
  }
  in();
  clk();
  out();
  bool p = 0;
  for (int i = 0; i < 32; i++) {
    bool b = (v >> i) & 1 U;
    p ^= b;
    wb(b);
  }
  wb(p);
  digitalWrite(SWDIO, LOW);
  clk();
  clk();
  return true;
}
static bool apWrite(uint8_t a, uint32_t v) {
  out();
  wbits(req(true, false, a), 8);
  in();
  clk();
  uint8_t ack = rbits(3);
  if (ack != 1) {
    out();
    digitalWrite(SWDIO, LOW);
    clk();
    return false;
  }
  in();
  clk();
  out();
  bool p = 0;
  for (int i = 0; i < 32; i++) {
    bool b = (v >> i) & 1 U;
    p ^= b;
    wb(b);
  }
  wb(p);
  digitalWrite(SWDIO, LOW);
  clk();
  clk();
  return true;
}
static bool apRead(uint8_t a, uint32_t & v) {
  out();
  wbits(req(true, true, a), 8);
  in();
  clk();
  uint8_t ack = rbits(3);
  if (ack != 1) {
    out();
    digitalWrite(SWDIO, LOW);
    clk();
    return false;
  }
  uint32_t d = rbits(32);
  bool p = rb();
  clk();
  out();
  digitalWrite(SWDIO, LOW);
  if (!par(d, p)) return false;
  return dpRead(0x0C, v);
}
static bool memR32(uint32_t a, uint32_t & v) {
  if (!apWrite(4, a)) return false;
  return apRead(0x0C, v);
}
static bool memW32(uint32_t a, uint32_t v) {
  return apWrite(4, a) && apWrite(0x0C, v);
}
static bool memR16(uint32_t a, uint16_t & v) {
  if (a & 1) return false;
  if (!apWrite(0, 0x23000041 UL) || !apWrite(4, a)) return false;
  uint32_t d = 0;
  if (!apRead(0x0C, d)) return false;
  if (!apWrite(0, 0x23000052 UL)) return false;
  v = (a & 2) ? (d >> 16) : (d & 0xFFFF U);
  return true;
}
static bool memW16(uint32_t a, uint16_t v) {
  if (a & 1) return false;
  if (!apWrite(0, 0x23000041 UL) || !apWrite(4, a)) return false;
  uint32_t d = (a & 2) ? ((uint32_t) v << 16) : v;
  bool ok = apWrite(0x0C, d);
  return apWrite(0, 0x23000052 UL) && ok;
}
static bool connect() {
  swdInit();
  uint32_t v = 0;
  if (!dpRead(0, v) || !dpWrite(0, 0x1E UL) || !dpWrite(4, 0x50000000 UL)) return false;
  delayMicroseconds(100);
  if (!dpRead(4, v) || (v & 0xF0000000 UL) != 0xF0000000 UL) return false;
  return dpWrite(8, 0) && apWrite(0, 0x23000052 UL);
}
static bool halt() {
  return memW32(0xE000EDF0 UL, 0xA05F0003 UL);
}
static bool resume() {
  return memW32(0xE000EDF0 UL, 0xA05F0001 UL);
}
static bool unlock() {
  uint32_t sr = 0, cr = 0;
  if (!memR32(FLASH_SR, sr) || !memR32(FLASH_CR, cr) || sr & 1) return false;
  if (!(cr & 0x80)) return true;
  return memW32(FLASH_KEYR, 0x45670123 UL) && memW32(FLASH_KEYR, 0xCDEF89AB UL) && memR32(FLASH_CR, cr) && !(cr & 0x80);
}
static bool waitReady(uint32_t ms = 5000) {
  uint32_t sr = 0;
  unsigned long t = millis();
  while (true) {
    if (!memR32(FLASH_SR, sr)) return false;
    if (!(sr & 1)) return !(sr & 0x14);
    if (millis() - t >= ms) return false;
    delay(1);
  }
}
static bool erasePage(uint32_t a) {
  uint32_t cr = 0;
  if (!memR32(FLASH_CR, cr)) return false;
  cr = (cr & ~0x43 UL) | 2 UL;
  if (!memW32(FLASH_CR, cr) || !memW32(FLASH_AR, a) || !memW32(FLASH_CR, cr | 0x40 UL) || !waitReady()) return false;
  return memW32(FLASH_CR, cr & ~2 UL);
}
static bool programChunk(uint32_t a,
  const uint8_t * d, size_t n) {
  if ((a & 1) || !n || n > 256) return false;
  if (!apWrite(0, 0x23000041 UL)) return false;
  uint32_t cr = 0;
  if (!memR32(FLASH_CR, cr) || !memW32(FLASH_CR, cr | 1 UL)) return false;
  for (size_t i = 0; i < n; i += 2) {
    uint16_t h = d[i] | ((i + 1 < n) ? ((uint16_t) d[i + 1] << 8) : 0xFF00 U);
    if (!memW16(a + i, h) || !waitReady()) return false;
  }
  if (!memR32(FLASH_CR, cr)) return false;
  return memW32(FLASH_CR, cr & ~1 UL);
}
static bool verifyChunk(uint32_t a,
  const uint8_t * d, size_t n) {
  for (size_t i = 0; i < n; i += 2) {
    uint16_t e = d[i] | ((i + 1 < n) ? ((uint16_t) d[i + 1] << 8) : 0xFF00 U), v = 0;
    if (!memR16(a + i, v) || v != e) return false;
  }
  return true;
}
static uint32_t crc32(const uint8_t * d, size_t n) {
  uint32_t c = 0xFFFFFFFF UL;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int b = 0; b < 8; b++) c = (c & 1) ? ((c >> 1) ^ 0xEDB88320 UL) : (c >> 1);
  }
  return c ^ 0xFFFFFFFF UL;
}
static bool readExact(uint8_t * d, size_t n) {
  size_t i = 0;
  unsigned long t = millis();
  while (i < n) {
    if (Serial.available()) d[i++] = (uint8_t) Serial.read();
    else if (millis() - t > 10000) return false;
    else delay(1);
  }
  return true;
}
static bool num(const char * s, uint32_t & v) {
  if (!s || ! * s) return false;
  char * e = nullptr;
  v = (uint32_t) strtoul(s, & e, 0);
  return * e == 0;
}
static void command(char * line) {
  char * save = nullptr;
  char * c = strtok_r(line, " \t", & save);
  if (!c) return;
  if (!strcasecmp(c, "PING")) {
    Serial.println("PONG STM32-SWD");
    return;
  }
  if (!strcasecmp(c, "INFO")) {
    uint32_t id = 0, fs = 0;
    if (!connect() || !dpRead(0, id) || !memR32(FLASH_SIZE_REG, fs)) {
      Serial.println("ERR CONNECT");
      return;
    }
    fs &= 0xFFFF;
    Serial.printf("OK ID=0x%08lX FLASH_SIZE=%lu PAGE_SIZE=1024 BASE=0x08000000\n", (unsigned long) id, (unsigned long) fs * 1024 UL);
    return;
  }
  if (!strcasecmp(c, "ERASE")) {
    uint32_t a = 0, n = 0, fs = 0;
    if (!num(strtok_r(nullptr, " \t", & save), a) || !num(strtok_r(nullptr, " \t", & save), n) || !n || !connect() || !memR32(FLASH_SIZE_REG, fs) || !halt() || !unlock()) {
      Serial.println("ERR ERASE");
      return;
    }
    fs &= 0xFFFF;
    uint32_t end = FLASH_BASE + fs * 1024 UL;
    if (a < FLASH_BASE || a >= end || n > end - a) {
      Serial.println("ERR ERASE_RANGE");
      return;
    }
    uint32_t first = a & ~(PAGE_SIZE - 1 UL), last = (a + n - 1) & ~(PAGE_SIZE - 1 UL);
    for (uint32_t p = first; p <= last; p += PAGE_SIZE)
      if (!erasePage(p)) {
        Serial.printf("ERR ERASE 0x%08lX\n", (unsigned long) p);
        return;
      } Serial.println("OK ERASE");
    return;
  }
  if (!strcasecmp(c, "WRITE")) {
    uint32_t a = 0, n = 0, crc = 0, fs = 0;
    if (!num(strtok_r(nullptr, " \t", & save), a) || !num(strtok_r(nullptr, " \t", & save), n) || !num(strtok_r(nullptr, " \t", & save), crc) || !n || n > 256 || (a & 1)) {
      Serial.println("ERR WRITE_ARGS");
      return;
    }
    if (!connect() || !memR32(FLASH_SIZE_REG, fs) || a < FLASH_BASE || n > fs * 1024 UL || (a - FLASH_BASE) > fs * 1024 UL - n || !halt() || !unlock()) {
      Serial.println("ERR WRITE_RANGE");
      return;
    }
    uint8_t d[256];
    Serial.println("READY");
    if (!readExact(d, n)) {
      Serial.println("ERR DATA_TIMEOUT");
      return;
    }
    if (crc32(d, n) != crc) {
      Serial.println("ERR CRC");
      return;
    }
    if (!programChunk(a, d, n) || !verifyChunk(a, d, n)) {
      Serial.println("ERR VERIFY");
      return;
    }
    Serial.printf("OK WRITE 0x%08lX %lu\n", (unsigned long) a, (unsigned long) n);
    return;
  }
  if (!strcasecmp(c, "RESET")) {
    if (!connect() || !memW32(0xE000ED0C UL, 0x05FA0004 UL)) {
      Serial.println("ERR RESET");
      return;
    }
    delay(50);
    resume();
    Serial.println("OK RESET");
    return;
  }
  Serial.println("ERR UNKNOWN_COMMAND");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(SWCLK, OUTPUT);
  digitalWrite(SWCLK, LOW);
  out();
  digitalWrite(SWDIO, LOW);
  Serial.println("ESP32 STM32F1 SWD programmer");
  Serial.println("READY: PING INFO ERASE WRITE RESET");
}
void loop() {
  static char line[96];
  static size_t n = 0;
  while (Serial.available()) {
    char c = (char) Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[n] = 0;
      command(line);
      n = 0;
    } else if (n < sizeof(line) - 1) line[n++] = c;
    else {
      n = 0;
      Serial.println("ERR LINE_TOO_LONG");
    }
  }
}