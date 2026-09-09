/* ============================================================
   HỆ THỐNG BÃI ĐỖ XE THÔNG MINH - ESP32 + ARDUINO FRAMEWORK
   ============================================================
   PHẦN CỨNG:
     - OLED SSD1306 0.96" I2C:  SDA -> GPIO21   SCL -> GPIO22
     - 4 cảm biến IR ô đỗ:      GPIO32, GPIO33, GPIO25, GPIO26
     - 2 cảm biến IR cổng:      VAO=GPIO27   RA=GPIO14
     - 2 servo barrier:         VAO=GPIO12   RA=GPIO13
     - HC-SR04:                 TRIG=GPIO5   ECHO=GPIO18
     - Buzzer:                  GPIO19

   LƯU Ý CHÂN ESP32:
     - GPIO34-39 chỉ input, KHÔNG có pull-up nội -> không dùng
       cho cảm biến IR kiểu open-collector cần pull-up.
     - Các chân đã chọn ở trên đều hỗ trợ INPUT_PULLUP bình thường.

   THƯ VIỆN CẦN CÀI (xem platformio.ini):
     - Adafruit SSD1306
     - Adafruit GFX Library
     - ESP32Servo
   ============================================================ */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

/* ================= CẤU HÌNH CHUNG ================= */
#define TOTAL_SLOTS 4

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* ---- Cảm biến IR 4 ô đỗ (LOW = có xe) ---- */
const uint8_t SLOT_PIN[TOTAL_SLOTS] = {32, 33, 25, 26};

/* ---- Cảm biến IR cổng vào / ra (LOW = có xe) ---- */
#define GATE_IN_PIN   27
#define GATE_OUT_PIN  14

/* ---- Servo barrier ---- */
#define SERVO_IN_PIN   12
#define SERVO_OUT_PIN  13

Servo servoIn;
Servo servoOut;

#define SERVO_ANGLE_CLOSED  0     // độ - barrier đóng
#define SERVO_ANGLE_OPEN    90    // độ - barrier mở

#define BARRIER_OPEN_TIME_MS 4000

/* ---- HC-SR04 ---- */
#define TRIG_PIN 5
#define ECHO_PIN 18

#define NGUONG_CANH_BAO_XA   30   // cm
#define NGUONG_CANH_BAO_GAN  10   // cm

/* ---- Buzzer ---- */
#define BUZZER_PIN 19


/* =========================================================
   TRẠNG THÁI BARRIER - STATE MACHINE NON-BLOCKING
   ========================================================= */

enum BarrierState
{
    BARRIER_CLOSED = 0,
    BARRIER_OPENED,
    BARRIER_WAIT_CLEAR
};

BarrierState barrierInState  = BARRIER_CLOSED;
BarrierState barrierOutState = BARRIER_CLOSED;

unsigned long barrierInOpenTick  = 0;
unsigned long barrierOutOpenTick = 0;


/* =========================================================
   CẢM BIẾN Ô ĐỖ
   ========================================================= */

uint8_t IsSlotOccupied(uint8_t index)
{
    return (digitalRead(SLOT_PIN[index]) == LOW) ? 1 : 0;
}

uint8_t CountFreeSlots(uint8_t *slotStatus)
{
    uint8_t freeCount = 0;
    for (uint8_t i = 0; i < TOTAL_SLOTS; i++)
    {
        slotStatus[i] = IsSlotOccupied(i);
        if (slotStatus[i] == 0) freeCount++;
    }
    return freeCount;
}


/* =========================================================
   HC-SR04
   ========================================================= */

float HCSR04_ReadDistance(void)
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    /* pulseIn trả về thời gian (us) mức HIGH của Echo, timeout tính bằng us */
    long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);

    if (duration == 0)
    {
        return -1; // không đo được / quá xa / mất tín hiệu
    }

    return duration / 58.0f; // cm
}


/* =========================================================
   BUZZER
   ========================================================= */

void Buzzer_Update(float distance)
{
    if (distance < 0 || distance > NGUONG_CANH_BAO_XA)
    {
        digitalWrite(BUZZER_PIN, LOW);
        return;
    }

    if (distance <= NGUONG_CANH_BAO_GAN)
    {
        digitalWrite(BUZZER_PIN, HIGH);
        return;
    }

    /* 10-30cm: kêu ngắt quãng, càng gần càng nhanh
       (dùng delay ngắn, chấp nhận được vì rất nhanh) */
    unsigned long beepDelay = (unsigned long)(distance * 10);

    digitalWrite(BUZZER_PIN, HIGH);
    delay(50);
    digitalWrite(BUZZER_PIN, LOW);
    delay(beepDelay);
}


/* =========================================================
   BARRIER - NON-BLOCKING STATE MACHINE
   ========================================================= */

uint8_t IsCarAtGate(uint8_t pin)
{
    return (digitalRead(pin) == LOW) ? 1 : 0;
}

void Barrier_Update(
    uint8_t sensorPin,
    Servo &servo,
    BarrierState *state,
    unsigned long *openTick,
    uint8_t allowOpen
)
{
    unsigned long now = millis();

    if (*state == BARRIER_CLOSED)
    {
        if (IsCarAtGate(sensorPin) && allowOpen)
        {
            servo.write(SERVO_ANGLE_OPEN);
            *state = BARRIER_OPENED;
            *openTick = now;
        }
    }
    else if (*state == BARRIER_OPENED)
    {
        if ((now - *openTick) >= BARRIER_OPEN_TIME_MS)
        {
            servo.write(SERVO_ANGLE_CLOSED);
            *state = BARRIER_WAIT_CLEAR;
        }
    }
    else /* BARRIER_WAIT_CLEAR */
    {
        if (!IsCarAtGate(sensorPin))
        {
            *state = BARRIER_CLOSED;
        }
    }
}


/* =========================================================
   OLED - HIỂN THỊ TRẠNG THÁI BÃI ĐỖ
   (Adafruit_SSD1306 hỗ trợ đầy đủ ASCII, chữ thường, dấu câu
    - không còn giới hạn font như bên STM32 nữa)
   ========================================================= */

void DisplayParkingStatus(uint8_t freeCount, uint8_t *slotStatus, float distance)
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.println("BAI DO XE");

    display.setCursor(0, 12);
    display.printf("Con trong: %d/%d\n", freeCount, TOTAL_SLOTS);

    display.setCursor(0, 24);
    display.printf("1:%s 2:%s\n", slotStatus[0] ? "X" : "T", slotStatus[1] ? "X" : "T");

    display.setCursor(0, 34);
    display.printf("3:%s 4:%s\n", slotStatus[2] ? "X" : "T", slotStatus[3] ? "X" : "T");

    display.setCursor(0, 46);
    if (distance >= 0)
        display.printf("KC: %.0f cm\n", distance);
    else
        display.println("KC: --");

    display.setCursor(0, 56);
    if (freeCount == 0)
        display.println("HET CHO!");
    else if (distance >= 0 && distance <= NGUONG_CANH_BAO_GAN)
        display.println("DO QUA SAT!");
    else
        display.println("Binh thuong");

    display.display();
}


/* =========================================================
   SETUP
   ========================================================= */

uint8_t slotStatus[TOTAL_SLOTS];
unsigned long lastDisplayTick = 0;
const unsigned long DISPLAY_INTERVAL_MS = 200;

void setup()
{
    Serial.begin(115200);

    /* ---- Cảm biến IR ô đỗ ---- */
    for (uint8_t i = 0; i < TOTAL_SLOTS; i++)
    {
        pinMode(SLOT_PIN[i], INPUT_PULLUP);
    }

    /* ---- Cảm biến IR cổng ---- */
    pinMode(GATE_IN_PIN, INPUT_PULLUP);
    pinMode(GATE_OUT_PIN, INPUT_PULLUP);

    /* ---- HC-SR04 ---- */
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);

    /* ---- Buzzer ---- */
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    /* ---- Servo ---- */
    servoIn.attach(SERVO_IN_PIN);
    servoOut.attach(SERVO_OUT_PIN);
    servoIn.write(SERVO_ANGLE_CLOSED);
    servoOut.write(SERVO_ANGLE_CLOSED);

    /* ---- OLED (I2C) ---- */
    Wire.begin(SDA_PIN, SCL_PIN);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println("Khong tim thay OLED!");
        while (1) { delay(10); }
    }

    display.clearDisplay();
    display.display();

    delay(500); // ổn định trạng thái ban đầu trước khi vào loop
}


/* =========================================================
   LOOP
   ========================================================= */

void loop()
{
    unsigned long now = millis();

    /* ---- Đọc trạng thái ô đỗ liên tục (barrier cần freeCount mới nhất) ---- */
    uint8_t freeCount = CountFreeSlots(slotStatus);

    /* ---- Barrier cổng VAO - kiểm tra liên tục mỗi vòng lặp ---- */
    Barrier_Update(
        GATE_IN_PIN, servoIn,
        &barrierInState, &barrierInOpenTick,
        (freeCount > 0)
    );

    /* ---- Barrier cổng RA - kiểm tra liên tục mỗi vòng lặp ---- */
    Barrier_Update(
        GATE_OUT_PIN, servoOut,
        &barrierOutState, &barrierOutOpenTick,
        1
    );

    /* ---- OLED + HC-SR04 + Buzzer cập nhật mỗi 200ms ---- */
    if (now - lastDisplayTick >= DISPLAY_INTERVAL_MS)
    {
        lastDisplayTick = now;

        float distance = HCSR04_ReadDistance();

        Buzzer_Update(distance);

        DisplayParkingStatus(freeCount, slotStatus, distance);
    }
}