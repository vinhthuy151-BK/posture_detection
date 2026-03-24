/*
 * =======================================================================================
 * PROJECT: SMART POSTURE GUARD
 * AUTHOR:  [Ten cua ban] - HCMUT Telecommunications & Electronics
 * DATE:    2025-12-03
 * COMPILER: avr-gcc
 * MCU:      ATmega328P
 *
 * FUSE BITS CONFIGURATION (IMPORTANT):
 * - Low Fuse:  0xFF (External Crystal Oscillator > 8MHz)
 * - High Fuse: 0xDE
 * - Extended:  0xFD
 * =======================================================================================
 */

#define F_CPU 16000000UL // Tan so thach anh 16MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/* =======================================================================================
 * MODULE 1: HARDWARE DEFINITIONS (PIN MAPPING)
 * D?a trên sõ ð? chân IC ATmega328P (Package DIP-28)
 * ======================================================================================= */

// --- STATUS LED (Output) ---
// LED noi vao chan so 19 cua chip (PB5/SCK))
#define LED_DDR     DDRB
#define LED_PORT    PORTB
#define LED_PIN     PB5  

// --- BUZZER (Output - Active High) ---
// Buzzer noi vao chan so 5 cua chip (PD3/INT1)
#define BUZZER_DDR  DDRD
#define BUZZER_PORT PORTD
#define BUZZER_PIN  PD3 

// --- SENSOR ADXL345 (I2C Interface) ---
// SDA: Chan so 27 (PC4)
// SCL: Chan so 28 (PC5)
#define ADXL345_ADDR    0x53 // Device Address
#define ADXL_REG_POWER  0x2D // Power Control Register
#define ADXL_REG_DATA   0x32 // Data Register (X0)

// --- SYSTEM CONSTANTS ---
#define ANGLE_THRESHOLD_DEG  25.0f  // Goc nghieng cho phep
#define TIME_THRESHOLD_MS    3000   // Thoi gian cho phep sai tu the (3s)
#define FILTER_ALPHA         0.2f   // He so loc (0.0 -> 1.0)

/* =======================================================================================
 * MODULE 2: DATA STRUCTURES & GLOBAL VARIABLES
 * Qu?n l? tr?ng thái h? th?ng
 * ======================================================================================= */

typedef struct {
    float filtX;
    float filtY;
    float filtZ;
    double rollAngle;
} SensorState_t;

volatile int system_millis = 0; // Bien dem thoi gian he thong

// --- Prototypes ---
void SYSTEM_Init(void);
void DRIVER_I2C_Init(void);
void DRIVER_I2C_Start(void);
void DRIVER_I2C_Stop(void);
void DRIVER_I2C_Write(void);
uint8_t DRIVER_I2C_ReadACK(void);
uint8_t DRIVER_I2C_ReadNACK(void);
void SENSOR_Init(void);
void SENSOR_Read(int16_t *x, int16_t *y, int16_t *z);
uint32_t millis(void);

/* =======================================================================================
 * MODULE 3: MAIN APPLICATION LOOP
 * ======================================================================================= */
int main(void) {
    // --- 1. System Boot ---
    SYSTEM_Init();       // Khoi tao Timer, GPIO
    DRIVER_I2C_Init();   // Khoi tao giao tiep I2C
    SENSOR_Init();       // Cau hinh ADXL345
    sei(); // Enable Global Interrupts

    // Bien cuc bo (Stack memory)
    SensorState_t imu = {0, 0, 256, 0}; // Gia tri khoi tao mac dinh (Z=1g)
    int16_t rawX, rawY, rawZ;
    
    uint32_t last_sample_time = 0;
    uint32_t posture_bad_start_time = 0;
    uint32_t buzzer_toggle_time = 0;
    
    uint8_t is_slouching = 0; // Co bao dang sai tu the
    uint8_t alert_active = 0; // Co kich hoat canh bao

    // --- 2. Infinite Loop ---
    while (1) {
        uint32_t current_ms = millis();

        // ---------------------------------------------------------
        // TASK A: SENSOR ACQUISITION & PROCESSING (Rate: 10Hz)
        // ---------------------------------------------------------
        if (current_ms - last_sample_time >= 100) {
            last_sample_time = current_ms;

            // B1: Doc du lieu tho
            SENSOR_Read(&rawX, &rawY, &rawZ);

            // B2: Loc nhieu (Low Pass Filter - EMA)
            imu.filtX = (rawX * FILTER_ALPHA) + (imu.filtX * (1.0f - FILTER_ALPHA));
            imu.filtY = (rawY * FILTER_ALPHA) + (imu.filtY * (1.0f - FILTER_ALPHA));
            imu.filtZ = (rawZ * FILTER_ALPHA) + (imu.filtZ * (1.0f - FILTER_ALPHA));

            // B3: Tinh toan goc nghieng (Roll Angle)
            // Cong thuc: atan2(Y, Z) chuyen doi sang Do (Degrees)
            imu.rollAngle = fabs(atan2(imu.filtY, imu.filtZ) * 57.29578f);

            // B4: Kiem tra Logic
            if (imu.rollAngle > ANGLE_THRESHOLD_DEG) {
                if (is_slouching == 0) {
                    is_slouching = 1;
                    posture_bad_start_time = current_ms; // Bat dau dem gio
                } else {
                    // Neu da duy tri sai tu the qua thoi gian cho phep
                    if ((current_ms - posture_bad_start_time) > TIME_THRESHOLD_MS) {
                        alert_active = 1;
                    }
                }
            } else {
                // Tu the binh thuong -> Reset toan bo
                is_slouching = 0;
                alert_active = 0;
                
                // Tat ngay lap tuc
                LED_PORT &= ~(1 << LED_PIN);
                BUZZER_PORT &= ~(1 << BUZZER_PIN);
            }
        }

        // ---------------------------------------------------------
        // TASK B: ALERT OUTPUT CONTROL (Non-blocking)
        // Tao hieu ung Bip-Bip: 100ms ON, 100ms OFF
        // ---------------------------------------------------------
        if (alert_active) {
            if (current_ms - buzzer_toggle_time >= 100) { // Chu ky 100ms
                buzzer_toggle_time = current_ms;
                
                // Dao trang thai Bit
                LED_PORT ^= (1 << LED_PIN);
                BUZZER_PORT ^= (1 << BUZZER_PIN);
            }
        }
    }
    return 0;
}

/* =======================================================================================
 * MODULE 4: SYSTEM DRIVERS (Hardware Abstraction Layer)
 * ======================================================================================= */

// --- 4.1 System Initialization (Timer & GPIO) ---
void SYSTEM_Init(void) {
    // Cau hinh chan Output
    LED_DDR |= (1 << LED_PIN);
    BUZZER_DDR |= (1 << BUZZER_PIN);
    
    // Cau hinh Timer0 de dem millis() (Mode CTC)
    TCCR0A = (1 << WGM01);              // CTC Mode
    OCR0A = 249;                        // 16MHz / 64 prescaler / 1000Hz = 250 ticks (0-249)
    TIMSK0 |= (1 << OCIE0A);            // Enable Interrupt Compare Match A
    TCCR0B |= (1 << CS01) | (1 << CS00);// Prescaler 64, Start Timer
}

// ISR cho Timer0 - Tuong tu millis() cua Arduino
ISR(TIMER0_COMPA_vect) {
    system_millis++;
}

uint32_t millis(void) {
    uint32_t m;
    cli(); // Atomic access
    m = system_millis;
    sei();
    return m;
}

// --- 4.2 I2C / TWI Driver (Low Level Register) ---
void DRIVER_I2C_Init(void) {
    // SCL Clock = 100kHz
    // Form: SCL_freq = F_CPU / (16 + 2*TWBR*Prescaler)
    TWSR = 0x00; // Prescaler = 1
    TWBR = 72;   // ((16000000/100000)-16)/2 = 72
    TWCR = (1 << TWEN); // Enable TWI Module
}

void DRIVER_I2C_Start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))); // Wait for flag
}

void DRIVER_I2C_Stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void DRIVER_I2C_Write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t DRIVER_I2C_ReadACK(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // Send ACK
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t DRIVER_I2C_ReadNACK(void) {
    TWCR = (1 << TWINT) | (1 << TWEN); // Send NACK (Last byte)
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

// --- 4.3 Sensor Driver (ADXL345 Specific) ---
void SENSOR_Init(void) {
    DRIVER_I2C_Start();
    DRIVER_I2C_Write(ADXL345_ADDR << 1); // Write Address
    DRIVER_I2C_Write(ADXL_REG_POWER);
    DRIVER_I2C_Write(0x08);              // Measurement Mode
    DRIVER_I2C_Stop();
}

void SENSOR_Read(int16_t *x, int16_t *y, int16_t *z) {
    DRIVER_I2C_Start();
    DRIVER_I2C_Write(ADXL345_ADDR << 1);
    DRIVER_I2C_Write(ADXL_REG_DATA);
    
    DRIVER_I2C_Start();                  // Repeated Start
    DRIVER_I2C_Write((ADXL345_ADDR << 1) | 1); // Read Address

    *x = DRIVER_I2C_ReadACK() | (DRIVER_I2C_ReadACK() << 8);
    *y = DRIVER_I2C_ReadACK() | (DRIVER_I2C_ReadACK() << 8);
    *z = DRIVER_I2C_ReadACK() | (DRIVER_I2C_ReadNACK() << 8); // Ket thuc bang NACK
    DRIVER_I2C_Stop();
}
