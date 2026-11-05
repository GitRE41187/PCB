/*
  PCB_Test.ino - โปรแกรมเทสบอร์ด ESP32 Trainer (MyPCB)
  บอร์ดเป้าหมาย : ESP32 DevKit V1 แบบ 30 ขา (เลือก "ESP32 Dev Module" ใน Arduino IDE)
  Serial Monitor : 115200 baud แล้วพิมพ์คำสั่งจากเมนู (พิมพ์ ? เพื่อดูเมนู)
  ไลบรารี        : ใช้แค่ Wire และ SPI ที่มากับ ESP32 core ไม่ต้องลงเพิ่ม

  สำคัญ: บนบอร์ดนี้ไม่มีลายทองแดงต่อจาก ESP32 ไปยังวงจรต่าง ๆ เลย ทุกจุดต้องต่อด้วย
  สายจัมเปอร์เอง ค่าด้านล่างเป็นแค่ค่าเริ่มต้น ถ้าต่อสายต่างออกไปให้แก้ตรงนี้ที่เดียว

  ขาที่ไม่ควรเปลี่ยนไปใช้เป็นอินพุตปุ่มกด:
    IO34/35/36/39 ไม่มี pull-up ในตัว และบนบอร์ดก็ไม่มี pull-up ที่เส้นปุ่ม
    IO2 ต่อกับ LED บน DevKit ทำให้อ่านเป็น LOW เมื่อใช้ pull-up ภายใน
*/

#include <Wire.h>
#include <SPI.h>
#include "driver/gpio.h"

// ---------- การต่อสาย: ขา ESP32 -> header บนบอร์ด ----------
const uint8_t PIN_SDA = 21;       // H4.2
const uint8_t PIN_SCL = 22;       // H4.1

const uint8_t PIN_ADC_CLK = 18;   // H12.1
const uint8_t PIN_ADC_MISO = 19;  // H12.2
const uint8_t PIN_ADC_MOSI = 23;  // H12.3
const uint8_t PIN_ADC_CS = 5;     // H12.4

const uint8_t PIN_SR_CLK = 16;    // H8.1
const uint8_t PIN_SR_LATCH = 17;  // H8.2
const uint8_t PIN_SR_DATA = 2;    // H8.3

const uint8_t PIN_LCD_RS = 14;    // H3.1
const uint8_t PIN_LCD_EN = 27;    // H3.2
const uint8_t PIN_LCD_D4 = 26;    // H3.3
const uint8_t PIN_LCD_D5 = 25;    // H3.4
const uint8_t PIN_LCD_D6 = 33;    // H3.5
const uint8_t PIN_LCD_D7 = 32;    // H3.6

// H13.1..4 = เน็ต SW1..SW4 = ปุ่ม SW3, SW4, SW5, SW6 บนบอร์ด (ชื่อเน็ตกับชื่อปุ่มไม่ตรงกัน)
const uint8_t PIN_SW[4] = {13, 12, 4, 15};
const char *const NAME_SW[4] = {"SW3 (H13.1)", "SW4 (H13.2)", "SW5 (H13.3)", "SW6 (H13.4)"};

// ปุ่มที่ต่อกับ PCF8574 โดยตรงบนบอร์ด: SW1 = P6, SW2 = P7
const char *const NAME_PCF_SW[2] = {"SW1 (PCF P6)", "SW2 (PCF P7)"};

// ขาที่ใช้เช็กลัดวงจร เอา IO2 ไว้ตัวแรก เพราะอ่านค่าเองไม่ได้ ทำได้แค่ขับ LOW ไปหาขาอื่น
// ไม่รวม IO1/IO3 (UART ของ USB) และ IO34/35/36/39 (ไม่มี pull-up ให้เช็ก)
const uint8_t SHORT_PINS[] = {2, 13, 12, 14, 27, 26, 25, 33, 32, 15, 4, 16, 17, 5, 18, 19, 21, 22, 23};

// ---------- ค่าคงที่ของการทดสอบ ----------
const uint32_t ASK_TIMEOUT_MS = 60000;
const uint32_t PRESS_TIMEOUT_MS = 15000;
const uint32_t RELEASE_TIMEOUT_MS = 5000;
const uint16_t ADC_HIGH_THRESHOLD = 3900;  // VREF = +3V3 ดังนั้นต่อ +3V3 ต้องได้เกือบ 4095
const float ADS_HIGH_THRESHOLD_V = 3.0;
const float ADS_LSB_V = 4.096 / 32768.0;   // ช่วง PGA +-4.096 V

enum Result : uint8_t { R_NOT_RUN, R_PASS, R_FAIL, R_SKIP };
enum Answer : uint8_t { A_YES, A_NO, A_SKIP };
enum TestId : uint8_t { T_BUS, T_SHORT, T_SCAN, T_PCF, T_SR, T_ADC, T_SW, T_LCD, T_LCDI2C, T_OLED, T_ADS, T_COUNT };

const char *const TEST_NAME[T_COUNT] = {
  "I2C bus + ไฟ +3V3 (R2, R3)",
  "ลัดวงจรระหว่างขา GPIO",
  "สแกน I2C",
  "PCF8574 U3 + LED1-4 + SW1-2",
  "74HC595 U1 + LED5-12",
  "MCP3208 U2 + AIN0-7",
  "ปุ่ม SW3-6",
  "LCD1602 DS1 (4 บิต)",
  "จอ LCD I2C ที่ J4",
  "จอ OLED ที่ J3",
  "ADS1115 ที่ J5",
};

struct AdcFrame {
  uint16_t value;
  bool valid;
  uint8_t rx0;
  uint8_t rx1;
};

typedef uint8_t (*ButtonReadFn)();
typedef void (*LcdSendFn)(uint8_t nibble, bool rs);

Result results[T_COUNT];
bool i2cOn = false;
bool spiOn = false;
uint8_t addrPcf = 0;
uint8_t addrLcdI2c = 0;
uint8_t addrOled = 0;
uint8_t addrAds = 0;

// ---------- ตัวช่วยทั่วไป ----------
void setResult(TestId id, Result r) {
  results[id] = r;
  const char *text = r == R_PASS ? "ผ่าน" : r == R_FAIL ? "ไม่ผ่าน" : r == R_SKIP ? "ข้าม" : "ยังไม่ทดสอบ";
  Serial.printf("==> %s : %s\n\n", TEST_NAME[id], text);
}

void title(TestId id) {
  Serial.printf("\n--- %s ---\n", TEST_NAME[id]);
}

void flushInput() {
  while (Serial.available()) Serial.read();
}

Answer ask(const char *question) {
  flushInput();
  Serial.printf("%s  [y = ใช่/ต่อแล้ว, n = ไม่, s = ข้าม]\n", question);
  uint32_t start = millis();
  while (millis() - start < ASK_TIMEOUT_MS) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'y' || c == 'Y') return A_YES;
      if (c == 'n' || c == 'N') return A_NO;
      if (c == 's' || c == 'S') return A_SKIP;
    }
    delay(5);
  }
  Serial.println("หมดเวลา ถือว่าข้าม");
  return A_SKIP;
}

Result visualResult(const char *question, const char *hintIfNo) {
  Answer a = ask(question);
  if (a == A_YES) return R_PASS;
  if (a == A_SKIP) return R_SKIP;
  Serial.println(hintIfNo);
  return R_FAIL;
}

void setPinsIdle() {
  const uint8_t outputs[] = {PIN_SR_CLK, PIN_SR_LATCH, PIN_SR_DATA, PIN_LCD_RS, PIN_LCD_EN,
                             PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7};
  for (uint8_t p : outputs) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  for (uint8_t p : PIN_SW) pinMode(p, INPUT_PULLUP);
  if (!spiOn) {
    pinMode(PIN_ADC_CS, OUTPUT);
    digitalWrite(PIN_ADC_CS, HIGH);
  }
}

void i2cBegin() {
  if (i2cOn) return;
  Wire.begin(PIN_SDA, PIN_SCL, 100000);
  i2cOn = true;
}

void i2cEnd() {
  if (!i2cOn) return;
  Wire.end();
  i2cOn = false;
}

void spiBegin() {
  if (spiOn) return;
  pinMode(PIN_ADC_CS, OUTPUT);
  digitalWrite(PIN_ADC_CS, HIGH);
  SPI.begin(PIN_ADC_CLK, PIN_ADC_MISO, PIN_ADC_MOSI, -1);
  // เปิด pull-up ที่ MISO เพื่อแยกแยะ "ชิปไม่ตอบ" (ค้าง HIGH) ออกจาก "ชิปตอบ" (null bit = 0)
  gpio_pullup_en((gpio_num_t)PIN_ADC_MISO);
  spiOn = true;
}

void spiEnd() {
  if (!spiOn) return;
  SPI.end();
  spiOn = false;
}

// ---------- 1. I2C bus + ไฟ +3V3 ----------
void testBus() {
  title(T_BUS);
  i2cEnd();
  // pull-down ภายใน (~45k) แพ้ R2/R3 (10k) ถ้ามีไฟ +3V3 และต่อ H4 แล้วจะอ่านได้ HIGH
  // ถ้าไม่มีไฟหรือไม่ได้ต่อสาย จะอ่านได้ LOW แน่นอน ไม่ลอย
  pinMode(PIN_SDA, INPUT_PULLDOWN);
  pinMode(PIN_SCL, INPUT_PULLDOWN);
  delay(5);
  bool sdaHigh = digitalRead(PIN_SDA);
  bool sclHigh = digitalRead(PIN_SCL);
  Serial.printf("SDA = %s, SCL = %s\n", sdaHigh ? "HIGH" : "LOW", sclHigh ? "HIGH" : "LOW");
  bool ok = sdaHigh && sclHigh;
  if (!ok) {
    Serial.println("เส้น I2C ไม่ขึ้น HIGH: เช็กสาย H4, ไฟ +3V3 (H2.1) และ R2/R3");
  } else {
    pinMode(PIN_SCL, OUTPUT);
    digitalWrite(PIN_SCL, LOW);
    delay(1);
    bool shorted = digitalRead(PIN_SDA) == LOW;
    pinMode(PIN_SCL, INPUT_PULLDOWN);
    if (shorted) {
      Serial.println("SDA กับ SCL ลัดกัน");
      ok = false;
    }
  }
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  setResult(T_BUS, ok ? R_PASS : R_FAIL);
}

// ---------- 2. ลัดวงจรระหว่างขา GPIO ----------
void testShort() {
  title(T_SHORT);
  Serial.println("ห้ามกดปุ่มระหว่างทดสอบ (จอ LCD อาจแสดงตัวอักษรมั่ว เป็นเรื่องปกติของการทดสอบนี้)");
  i2cEnd();
  spiEnd();
  const uint8_t n = sizeof(SHORT_PINS);
  bool stuck[n];
  bool ok = true;
  for (uint8_t i = 0; i < n; i++) pinMode(SHORT_PINS[i], INPUT_PULLUP);
  delay(5);
  for (uint8_t i = 0; i < n; i++) {
    stuck[i] = SHORT_PINS[i] != 2 && digitalRead(SHORT_PINS[i]) == LOW;
    if (stuck[i]) {
      Serial.printf("IO%d ถูกดึงลง LOW: ลัดลง GND หรือไปต่อกับชิปที่ยังไม่มีไฟ\n", SHORT_PINS[i]);
      ok = false;
    }
  }
  for (uint8_t i = 0; i < n; i++) {
    if (stuck[i]) continue;
    pinMode(SHORT_PINS[i], OUTPUT);
    digitalWrite(SHORT_PINS[i], LOW);
    delayMicroseconds(200);
    for (uint8_t j = i + 1; j < n; j++) {
      if (stuck[j] || SHORT_PINS[j] == 2) continue;
      if (digitalRead(SHORT_PINS[j]) == LOW) {
        Serial.printf("IO%d ลัดกับ IO%d\n", SHORT_PINS[i], SHORT_PINS[j]);
        ok = false;
      }
    }
    pinMode(SHORT_PINS[i], INPUT_PULLUP);
    delayMicroseconds(200);
  }
  setPinsIdle();
  setResult(T_SHORT, ok ? R_PASS : R_FAIL);
}

// ---------- 3. สแกน I2C ----------
void scanI2c(bool verbose) {
  i2cBegin();
  addrPcf = addrLcdI2c = addrOled = addrAds = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() != 0) continue;
    const char *what = "ไม่รู้จัก";
    if (a >= 0x20 && a <= 0x26) {
      if (!addrPcf) addrPcf = a;
      what = "PCF8574 U3";
    } else if (a == 0x27 || a == 0x3F) {
      if (!addrLcdI2c) addrLcdI2c = a;
      what = "จอ LCD I2C ที่ J4 (ถ้า H5 ต่อไว้ครบทั้ง 3 ขาเป็น HIGH อาจเป็น U3)";
    } else if (a == 0x3C || a == 0x3D) {
      addrOled = a;
      what = "OLED ที่ J3";
    } else if (a >= 0x48 && a <= 0x4B) {
      addrAds = a;
      what = "ADS1115 ที่ J5";
    }
    if (verbose) Serial.printf("  พบ 0x%02X : %s\n", a, what);
  }
}

void testScan() {
  title(T_SCAN);
  scanI2c(true);
  if (!addrPcf) {
    Serial.println("ไม่พบ U3: H5 (A0-A2) ลอยอยู่บนบอร์ด ต้องต่อลง GND ทั้ง 3 ขา (ได้ 0x20)");
    Serial.println("ถ้าต่อแล้วยังไม่เจอ ให้เช็กทิศ IC, ไฟที่ U3.16 และการบัดกรีขา 14, 15");
  }
  setResult(T_SCAN, addrPcf ? R_PASS : R_FAIL);
}

// ---------- ปุ่มกด (ใช้ร่วมกันระหว่าง SW1-2 และ SW3-6) ----------
Result runButtons(const char *const names[], uint8_t count, ButtonReadFn readFn) {
  uint8_t idle = readFn();
  if (idle != 0) {
    Serial.printf("ปุ่มถูกกดค้างตั้งแต่ก่อนเริ่ม (mask 0x%02X): ลัดลง GND หรือปุ่มเสีย\n", idle);
    return R_FAIL;
  }
  bool ok = true;
  for (uint8_t i = 0; i < count; i++) {
    flushInput();
    Serial.printf("กดปุ่ม %s ภายใน %lu วินาที (พิมพ์ s เพื่อข้าม)\n", names[i], (unsigned long)(PRESS_TIMEOUT_MS / 1000));
    uint32_t start = millis();
    uint8_t mask = 0;
    while (millis() - start < PRESS_TIMEOUT_MS) {
      if (Serial.available() && (Serial.read() | 0x20) == 's') return R_SKIP;
      mask = readFn();
      if (mask) {
        delay(20);
        mask = readFn();
        if (mask) break;
      }
      delay(5);
    }
    if (mask == 0) {
      Serial.println("  ไม่เห็นการกด: เช็กสาย, การบัดกรีปุ่ม และ GND");
      ok = false;
      continue;
    }
    if (mask != (1 << i)) {
      Serial.printf("  ได้ mask 0x%02X แทนที่จะเป็น 0x%02X: ต่อสลับหรือลัดกับปุ่มอื่น\n", mask, 1 << i);
      ok = false;
    } else {
      Serial.println("  OK");
    }
    start = millis();
    while (readFn() != 0) {
      if (millis() - start > RELEASE_TIMEOUT_MS) {
        Serial.println("  ปล่อยแล้วแต่ยังอ่านว่ากดอยู่");
        ok = false;
        break;
      }
      delay(5);
    }
    delay(50);
  }
  return ok ? R_PASS : R_FAIL;
}

// ---------- 4. PCF8574 U3 ----------
bool pcfWrite(uint8_t v) {
  Wire.beginTransmission(addrPcf);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

int pcfRead() {
  if (Wire.requestFrom(addrPcf, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

uint8_t readPcfButtons() {
  int v = pcfRead();
  if (v < 0) return 0;
  uint8_t mask = 0;
  if (!(v & 0x40)) mask |= 1;
  if (!(v & 0x80)) mask |= 2;
  return mask;
}

void testPcf() {
  title(T_PCF);
  scanI2c(false);
  if (!addrPcf) {
    Serial.println("ไม่พบ U3 ที่ 0x20-0x26 (ดูคำแนะนำในหัวข้อสแกน I2C)");
    setResult(T_PCF, R_FAIL);
    return;
  }
  Serial.printf("U3 อยู่ที่ 0x%02X (A2 A1 A0 = %d %d %d)\n", addrPcf, (addrPcf >> 2) & 1, (addrPcf >> 1) & 1, addrPcf & 1);

  bool ok = pcfWrite(0xFF);
  delay(2);
  int v = pcfRead();
  if (v < 0) {
    Serial.println("อ่านค่าจาก U3 ไม่ได้: สัญญาณ I2C ไม่เสถียร เช็ก H5 ว่าไม่ลอย และสาย H4");
    ok = false;
  } else if (v != 0xFF) {
    Serial.printf("ปล่อยทุกขาแล้วอ่านได้ 0x%02X แทนที่จะเป็น 0xFF: มีขาถูกดึงลง GND\n", v);
    ok = false;
  }
  // LED ต่อแบบ +3V3 -> LED -> R -> Pn ดังนั้น P เป็น LOW คือ LED ติด
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t out = ~(1 << i);
    pcfWrite(out);
    Serial.printf("  P%d -> LED%d ติด\n", i, i + 1);
    delay(500);
    v = pcfRead();
    if (v != out) {
      Serial.printf("  อ่านกลับได้ 0x%02X แทนที่จะเป็น 0x%02X: ขา P ลัดกัน\n", v & 0xFF, out);
      ok = false;
    }
  }
  pcfWrite(0xF0);
  delay(800);
  pcfWrite(0xFF);

  Result leds = visualResult("LED1, 2, 3, 4 ติดเรียงทีละดวง แล้วติดพร้อมกันใช่ไหม?",
                             "เช็กขั้ว LED, R7-R10 และไฟ +3V3");
  Result buttons = runButtons(NAME_PCF_SW, 2, readPcfButtons);
  if (!ok || leds == R_FAIL || buttons == R_FAIL) setResult(T_PCF, R_FAIL);
  else if (leds == R_SKIP || buttons == R_SKIP) setResult(T_PCF, R_SKIP);
  else setResult(T_PCF, R_PASS);
}

// ---------- 5. 74HC595 U1 ----------
void srWrite(uint8_t v) {
  digitalWrite(PIN_SR_LATCH, LOW);
  shiftOut(PIN_SR_DATA, PIN_SR_CLK, MSBFIRST, v);  // บิต 0 = Q1
  digitalWrite(PIN_SR_LATCH, HIGH);
}

void testSr() {
  title(T_SR);
  setPinsIdle();
  Serial.println("ต้องต่อ H9 -> H10 ครบ 8 เส้นก่อน (Q1->D1 ... Q8->D8) ไม่งั้น LED จะไม่ติด");
  srWrite(0);
  delay(300);
  for (uint8_t i = 0; i < 8; i++) {
    srWrite(1 << i);
    Serial.printf("  Q%d -> LED%d ติด\n", i + 1, i + 5);
    delay(350);
  }
  srWrite(0xFF);
  delay(800);
  srWrite(0xAA);
  delay(500);
  srWrite(0x55);
  delay(500);
  srWrite(0);
  setResult(T_SR, visualResult("LED5-12 ไล่ทีละดวงตามลำดับ แล้วติดหมด แล้วสลับคู่-คี่ใช่ไหม?",
                               "เช็กสาย H8, สาย H9->H10, ไฟ U1.16 และขั้ว LED"));
}

// ---------- 6. MCP3208 U2 ----------
AdcFrame adcRead(uint8_t ch) {
  AdcFrame f;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_ADC_CS, LOW);
  f.rx0 = SPI.transfer(0x06 | (ch >> 2));
  f.rx1 = SPI.transfer((ch & 0x03) << 6);
  uint8_t rx2 = SPI.transfer(0x00);
  digitalWrite(PIN_ADC_CS, HIGH);
  SPI.endTransaction();
  delayMicroseconds(30);  // ให้ pull-up ที่ MISO ดึงกลับ HIGH ก่อนเฟรมถัดไป
  f.value = ((uint16_t)(f.rx1 & 0x0F) << 8) | rx2;
  // ก่อนถึง null bit MISO ต้องเป็น Hi-Z (อ่านได้ 1 จาก pull-up) และ null bit ต้องเป็น 0
  f.valid = f.rx0 == 0xFF && (f.rx1 & 0x10) == 0;
  return f;
}

void testAdc() {
  title(T_ADC);
  spiBegin();
  AdcFrame f = adcRead(0);
  if (!f.valid) {
    if (f.rx0 != 0xFF) Serial.println("MISO ค้าง LOW: เช็กไฟที่ U2.15, U2.16 และลัดวงจรที่ H12.2");
    else Serial.println("U2 ไม่ตอบ (MISO ค้าง HIGH): เช็กสาย H12, ทิศ IC, การบัดกรี");
    setResult(T_ADC, R_FAIL);
    return;
  }
  Serial.println("U2 ตอบ SPI ถูกต้อง (CLK, CS, MOSI, MISO ใช้ได้)");
  Serial.print("ค่าตอนนี้ (ขาที่ลอยจะอ่านได้ค่ามั่ว เป็นเรื่องปกติ):");
  for (uint8_t ch = 0; ch < 8; ch++) Serial.printf(" %u", adcRead(ch).value);
  Serial.println();

  bool ok = true;
  uint8_t tested = 0;
  char q[96];
  for (uint8_t ch = 0; ch < 8; ch++) {
    snprintf(q, sizeof(q), "ต่อสายจาก +3V3 (H2.1) ไป AIN%d (H11.%d) อย่างเดียว", ch, ch * 2 + 1);
    if (ask(q) != A_YES) break;
    adcRead(ch);  // เฟรมแรกหลังเปลี่ยนช่องเอาไว้ล้างค่าเก่าใน sample cap
    uint16_t v = adcRead(ch).value;
    bool pass = v >= ADC_HIGH_THRESHOLD;
    Serial.printf("  AIN%d = %u (%.2f V) %s\n", ch, v, v * 3.3 / 4095.0, pass ? "OK" : "ต่ำเกินไป");
    if (!pass) ok = false;
    tested++;
  }
  if (!ok) setResult(T_ADC, R_FAIL);
  else setResult(T_ADC, tested == 8 ? R_PASS : R_SKIP);
}

// ---------- 7. ปุ่ม SW3-6 ----------
uint8_t readSwButtons() {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (digitalRead(PIN_SW[i]) == LOW) mask |= 1 << i;
  }
  return mask;
}

void testSw() {
  title(T_SW);
  setPinsIdle();
  delay(5);
  setResult(T_SW, runButtons(NAME_SW, 4, readSwButtons));
}

// ---------- จอ HD44780 (ใช้ร่วมกันระหว่าง DS1 และจอ I2C) ----------
void lcdByte(LcdSendFn send, uint8_t v, bool rs) {
  send(v >> 4, rs);
  send(v & 0x0F, rs);
  delayMicroseconds(60);
}

void lcdInit(LcdSendFn send) {
  delay(50);
  send(0x03, false);
  delay(5);
  send(0x03, false);
  delayMicroseconds(150);
  send(0x03, false);
  delayMicroseconds(150);
  send(0x02, false);
  delayMicroseconds(150);
  lcdByte(send, 0x28, false);  // 4 บิต 2 บรรทัด
  lcdByte(send, 0x0C, false);  // เปิดจอ ปิดเคอร์เซอร์
  lcdByte(send, 0x01, false);  // ล้างจอ
  delay(3);
  lcdByte(send, 0x06, false);  // เลื่อนเคอร์เซอร์ไปทางขวา
}

void lcdPrintLine(LcdSendFn send, uint8_t line, const char *text) {
  lcdByte(send, 0x80 | (line ? 0x40 : 0x00), false);
  while (*text) lcdByte(send, *text++, true);
}

// ---------- 8. LCD1602 DS1 แบบ 4 บิต ----------
void lcdGpioSend(uint8_t nibble, bool rs) {
  digitalWrite(PIN_LCD_RS, rs);
  digitalWrite(PIN_LCD_D4, nibble & 0x01);
  digitalWrite(PIN_LCD_D5, nibble & 0x02);
  digitalWrite(PIN_LCD_D6, nibble & 0x04);
  digitalWrite(PIN_LCD_D7, nibble & 0x08);
  delayMicroseconds(1);
  digitalWrite(PIN_LCD_EN, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_LCD_EN, LOW);
  delayMicroseconds(50);
}

void testLcd() {
  title(T_LCD);
  setPinsIdle();
  Serial.println("ต้องมีไฟ +5V ที่ H19.1 และหมุน RV1 เพื่อปรับความเข้มถ้าไม่เห็นตัวอักษร");
  lcdInit(lcdGpioSend);
  lcdPrintLine(lcdGpioSend, 0, "ABCDEFGHIJKLMNOP");
  lcdPrintLine(lcdGpioSend, 1, "0123456789 TEST!");
  setResult(T_LCD, visualResult("จอแสดง ABCDEFGHIJKLMNOP / 0123456789 TEST! ถูกต้องใช่ไหม?",
                                "ไม่มีอะไรเลย = ไฟ +5V หรือ RV1, สี่เหลี่ยมทึบบรรทัดเดียว = สาย RS/EN/D4-D7, ตัวอักษรเพี้ยน = D4-D7 สลับหรือลัด"));
}

// ---------- 9. จอ LCD I2C ที่ J4 ----------
void lcdI2cSend(uint8_t nibble, bool rs) {
  // ผังขามาตรฐานของ backpack: P0 = RS, P1 = RW, P2 = EN, P3 = ไฟจอ, P4-P7 = D4-D7
  uint8_t b = (nibble << 4) | 0x08 | (rs ? 0x01 : 0x00);
  Wire.beginTransmission(addrLcdI2c);
  Wire.write(b | 0x04);
  Wire.endTransmission();
  Wire.beginTransmission(addrLcdI2c);
  Wire.write(b);
  Wire.endTransmission();
  delayMicroseconds(50);
}

void testLcdI2c() {
  title(T_LCDI2C);
  scanI2c(false);
  if (!addrLcdI2c) {
    Serial.println("ไม่พบจอที่ 0x27 หรือ 0x3F (ถ้าไม่ได้เสียบจอ ข้ามได้เลย)");
    setResult(T_LCDI2C, R_SKIP);
    return;
  }
  Serial.printf("จออยู่ที่ 0x%02X\nJ4 จ่ายไฟแค่ +3V3 จอรุ่น 5V อาจจางมาก ให้หมุนปรับความเข้มที่หลังจอ\n", addrLcdI2c);
  lcdInit(lcdI2cSend);
  lcdPrintLine(lcdI2cSend, 0, "LCD I2C ON J4");
  lcdPrintLine(lcdI2cSend, 1, "0123456789 TEST!");
  setResult(T_LCDI2C, visualResult("จอ I2C แสดง LCD I2C ON J4 / 0123456789 TEST! ใช่ไหม?",
                                   "ถ้าเจอที่อยู่แต่ไม่มีตัวอักษร ให้ปรับ contrast หลังจอก่อน"));
}

// ---------- 10. จอ OLED ที่ J3 ----------
void oledCmd(uint8_t c) {
  Wire.beginTransmission(addrOled);
  Wire.write(0x00);
  Wire.write(c);
  Wire.endTransmission();
}

// ใช้ page mode และเขียน 132 คอลัมน์ เพื่อให้ใช้ได้ทั้ง SSD1306 และ SH1106
void oledFill(bool checker) {
  for (uint8_t page = 0; page < 8; page++) {
    oledCmd(0xB0 | page);
    oledCmd(0x00);
    oledCmd(0x10);
    uint8_t col = 0;
    while (col < 132) {
      Wire.beginTransmission(addrOled);
      Wire.write(0x40);
      for (uint8_t i = 0; i < 16 && col < 132; i++, col++) {
        Wire.write(!checker || ((col / 8 + page) & 1) ? 0xFF : 0x00);
      }
      Wire.endTransmission();
    }
  }
}

void testOled() {
  title(T_OLED);
  scanI2c(false);
  if (!addrOled) {
    Serial.println("ไม่พบ OLED ที่ 0x3C หรือ 0x3D (ถ้าไม่ได้เสียบจอ ข้ามได้เลย)");
    setResult(T_OLED, R_SKIP);
    return;
  }
  const uint8_t init[] = {0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x02,
                          0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF};
  for (uint8_t c : init) oledCmd(c);
  oledFill(false);
  delay(1000);
  oledFill(true);
  setResult(T_OLED, visualResult("OLED ติดเต็มจอ 1 วินาที แล้วเป็นตารางหมากรุกใช่ไหม?",
                                 "เช็กลำดับขาของโมดูล: J3 = +3V3, GND, SCL, SDA"));
}

// ---------- 11. ADS1115 ที่ J5 ----------
bool adsWrite16(uint8_t reg, uint16_t v) {
  Wire.beginTransmission(addrAds);
  Wire.write(reg);
  Wire.write(v >> 8);
  Wire.write(v & 0xFF);
  return Wire.endTransmission() == 0;
}

int32_t adsRead16(uint8_t reg) {
  Wire.beginTransmission(addrAds);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return -1;
  if (Wire.requestFrom(addrAds, (uint8_t)2) != 2) return -1;
  uint16_t hi = Wire.read();
  return (hi << 8) | Wire.read();
}

float adsReadVolts(uint8_t ch) {
  // single-shot, MUX = ch เทียบ GND, PGA +-4.096 V, 128 SPS, ปิด comparator
  uint16_t config = 0x8000 | ((uint16_t)(4 + ch) << 12) | 0x0200 | 0x0100 | 0x0080 | 0x0003;
  if (!adsWrite16(0x01, config)) return NAN;
  delay(12);
  int32_t raw = adsRead16(0x00);
  if (raw < 0) return NAN;
  return (int16_t)raw * ADS_LSB_V;
}

void testAds() {
  title(T_ADS);
  scanI2c(false);
  if (!addrAds) {
    Serial.println("ไม่พบ ADS1115 ที่ 0x48-0x4B (ถ้าไม่ได้เสียบโมดูล ข้ามได้เลย)");
    setResult(T_ADS, R_SKIP);
    return;
  }
  Serial.printf("ADS1115 อยู่ที่ 0x%02X\n", addrAds);
  bool ok = true;
  uint8_t tested = 0;
  char q[96];
  for (uint8_t ch = 0; ch < 4; ch++) {
    snprintf(q, sizeof(q), "ต่อสายจาก +3V3 (H2.1) ไป ADS_A%d (H6.%d) อย่างเดียว", ch, ch * 2 + 1);
    if (ask(q) != A_YES) break;
    float v = adsReadVolts(ch);
    bool pass = !isnan(v) && v >= ADS_HIGH_THRESHOLD_V;
    Serial.printf("  A%d = %.3f V %s\n", ch, v, pass ? "OK" : "ต่ำเกินไปหรืออ่านไม่ได้");
    if (!pass) ok = false;
    tested++;
  }
  if (!ok) setResult(T_ADS, R_FAIL);
  else setResult(T_ADS, tested == 4 ? R_PASS : R_SKIP);
}

// ---------- ดูค่าสดทุกอินพุต ----------
void liveMonitor() {
  Serial.println("\nดูค่าสด กดปุ่มใดก็ได้ในช่อง Serial เพื่อออก");
  setPinsIdle();
  spiBegin();
  scanI2c(false);
  flushInput();
  while (!Serial.available()) {
    Serial.print("MCP3208:");
    AdcFrame f = adcRead(0);
    if (f.valid) {
      for (uint8_t ch = 0; ch < 8; ch++) Serial.printf(" %4u", adcRead(ch).value);
    } else {
      Serial.print(" ไม่ตอบ");
    }
    if (addrAds) {
      Serial.print(" | ADS:");
      for (uint8_t ch = 0; ch < 4; ch++) Serial.printf(" %.2f", adsReadVolts(ch));
    }
    Serial.printf(" | SW3-6 mask: 0x%X", readSwButtons());
    if (addrPcf) Serial.printf(" | SW1-2 mask: 0x%X", readPcfButtons());
    Serial.println();
    delay(500);
  }
  flushInput();
}

// ---------- เมนู ----------
void printWiring() {
  Serial.println("\n===== การต่อสายก่อนเทส =====");
  Serial.println("ไฟ (เสียบ USB อย่างเดียว ไม่ต้องมีโมดูลเรกกูเลเตอร์):");
  Serial.println("  H15.1 (ESP 3V3) -> H17.1 (+3V3)");
  Serial.println("  H14.1 (ESP VIN) -> H19.1 (+5V สำหรับ LCD)");
  Serial.println("  ถ้าเสียบโมดูล 3.3V ที่ H17 ไม่ต้องต่อเส้น H15.1 -> H17.1");
  Serial.println("  เว้น H2.1 ไว้ ใช้เป็นจุด +3V3 ตอนเทส AIN และ ADS");
  Serial.println("  GND ของ ESP ต่อกับบอร์ดอยู่แล้วที่ J1.2");
  Serial.println("I2C    : IO22 -> H4.1 (SCL), IO21 -> H4.2 (SDA)");
  Serial.println("PCF    : H5.1 -> H6.4, H5.2 -> H6.6, H5.3 -> H6.8 (ขาคู่ของ H6 คือ GND) ได้ที่อยู่ 0x20");
  Serial.println("MCP3208: IO18 -> H12.1, IO19 -> H12.2, IO23 -> H12.3, IO5 -> H12.4");
  Serial.println("74HC595: IO16 -> H8.1, IO17 -> H8.2, IO2 -> H8.3");
  Serial.println("LED8   : H9.1 -> H10.1 ... H9.8 -> H10.8 (8 เส้น)");
  Serial.println("LCD    : IO14 -> H3.1, IO27 -> H3.2, IO26 -> H3.3, IO25 -> H3.4, IO33 -> H3.5, IO32 -> H3.6");
  Serial.println("ปุ่ม     : IO13 -> H13.1, IO12 -> H13.2, IO4 -> H13.3, IO15 -> H13.4");
  Serial.println("ตำแหน่ง IO: H14 = VIN GND 13 12 14 27 26 25 33 32 35 34 39 36 EN");
  Serial.println("           H15 = 3V3 GND 15 2 4 16 17 5 18 19 21 3 1 22 23");
}

void printMenu() {
  Serial.println("\n===== PCB Test: ESP32 Trainer =====");
  Serial.println("a = ทดสอบทั้งหมดตามลำดับ");
  for (uint8_t i = 0; i < T_COUNT; i++) {
    Serial.printf("%c = %s\n", 'A' + i, TEST_NAME[i]);
  }
  Serial.println("m = ดูค่าสด (ADC, ปุ่ม)");
  Serial.println("r = สรุปผล");
  Serial.println("w = ดูการต่อสาย");
  Serial.println("? = เมนูนี้");
  Serial.println("(ตัวพิมพ์ใหญ่ A-K คือการเลือกทดสอบทีละข้อ)");
}

void printSummary() {
  Serial.println("\n===== สรุปผล =====");
  uint8_t fails = 0;
  for (uint8_t i = 0; i < T_COUNT; i++) {
    Result r = results[i];
    const char *text = r == R_PASS ? "ผ่าน" : r == R_FAIL ? "ไม่ผ่าน" : r == R_SKIP ? "ข้าม" : "-";
    Serial.printf("[%-3s] %s\n", text, TEST_NAME[i]);
    if (r == R_FAIL) fails++;
  }
  Serial.printf("ไม่ผ่าน %d รายการ\n", fails);
}

void runTest(uint8_t id) {
  switch (id) {
    case T_BUS: testBus(); break;
    case T_SHORT: testShort(); break;
    case T_SCAN: testScan(); break;
    case T_PCF: testPcf(); break;
    case T_SR: testSr(); break;
    case T_ADC: testAdc(); break;
    case T_SW: testSw(); break;
    case T_LCD: testLcd(); break;
    case T_LCDI2C: testLcdI2c(); break;
    case T_OLED: testOled(); break;
    case T_ADS: testAds(); break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  setPinsIdle();
  srWrite(0);  // 74HC595 เปิดเครื่องมาค่าสุ่ม เคลียร์ทิ้งก่อน
  printWiring();
  printMenu();
}

void loop() {
  if (!Serial.available()) {
    delay(10);
    return;
  }
  char c = Serial.read();
  if (c >= 'A' && c < 'A' + T_COUNT) {
    runTest(c - 'A');
    printSummary();
    return;
  }
  switch (c) {
    case 'a':
      for (uint8_t i = 0; i < T_COUNT; i++) runTest(i);
      printSummary();
      break;
    case 'm': liveMonitor(); break;
    case 'r': printSummary(); break;
    case 'w': printWiring(); break;
    case '?': printMenu(); break;
    default: break;
  }
}
