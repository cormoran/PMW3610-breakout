// This code is based on https://github.com/inorichi/zmk-pmw3610-driver
#include "Adafruit_TinyUSB.h"

// xiao-ble pin assignment
// Note: to use NFC2 as GPIO, you need to set `-DCONFIG_NFCT_PINS_AS_GPIOS` to
// xiaonRF52840.build.extra_flags in board definition
//       e.g. For Mac,
//       /Users/<user>/Library/Arduino15/packages/Seeeduino/hardware/nrf52/1.1.8/boards.txt
//            Change corresponding line to
//            `xiaonRF52840.build.extra_flags=-DNRF52840_XXAA -DCONFIG_NFCT_PINS_AS_GPIOS {build.flags.usb}`

#define SCLK A5
#define SDIO A4
#define MOTION PIN_NFC2
// #define NCS // not used

/* Sensor registers (addresses) */
#define PMW3610_REG_PRODUCT_ID 0x00
#define PMW3610_REG_REVISION_ID 0x01
#define PMW3610_REG_MOTION 0x02
#define PMW3610_REG_DELTA_X_L 0x03
#define PMW3610_REG_DELTA_Y_L 0x04
#define PMW3610_REG_DELTA_XY_H 0x05
#define PMW3610_REG_SQUAL 0x06
#define PMW3610_REG_SHUTTER_HIGHER 0x07
#define PMW3610_REG_SHUTTER_LOWER 0x08
#define PMW3610_REG_PIX_MAX 0x09
#define PMW3610_REG_PIX_AVG 0x0A
#define PMW3610_REG_PIX_MIN 0x0B

#define PMW3610_REG_CRC0 0x0C
#define PMW3610_REG_CRC1 0x0D
#define PMW3610_REG_CRC2 0x0E
#define PMW3610_REG_CRC3 0x0F
#define PMW3610_REG_SELF_TEST 0x10

#define PMW3610_REG_PERFORMANCE 0x11
#define PMW3610_REG_MOTION_BURST 0x12

#define PMW3610_REG_RUN_DOWNSHIFT 0x1B
#define PMW3610_REG_REST1_PERIOD 0x1C
#define PMW3610_REG_REST1_DOWNSHIFT 0x1D
#define PMW3610_REG_REST2_PERIOD 0x1E
#define PMW3610_REG_REST2_DOWNSHIFT 0x1F
#define PMW3610_REG_REST3_PERIOD 0x20
#define PMW3610_REG_OBSERVATION 0x2D

#define PMW3610_REG_PIXEL_GRAB 0x35
#define PMW3610_REG_FRAME_GRAB 0x36

#define PMW3610_REG_POWER_UP_RESET 0x3A
#define PMW3610_REG_SHUTDOWN 0x3B

#define PMW3610_REG_SPI_CLK_ON_REQ 0x41
#define PMW3610_REG_RES_STEP 0x85

#define PMW3610_REG_NOT_REV_ID 0x3E
#define PMW3610_REG_NOT_PROD_ID 0x3F

#define PMW3610_REG_PRBS_TEST_CTL 0x47
#define PMW3610_REG_SPI_PAGE0 0x7F
#define PMW3610_REG_VCSEL_CTL 0x9E
#define PMW3610_REG_LSR_CONTROL 0x9F
#define PMW3610_REG_SPI_PAGE1 0xFF

/* Sensor identification values */
#define PMW3610_PRODUCT_ID 0x3E

/* Power-up register commands */
#define PMW3610_POWERUP_CMD_RESET 0x5A
#define PMW3610_POWERUP_CMD_WAKEUP 0x96

/* spi clock enable/disable commands */
#define PMW3610_SPI_CLOCK_CMD_ENABLE 0xBA
#define PMW3610_SPI_CLOCK_CMD_DISABLE 0xB5

/* write command bit position */
#define BIT(i) (1 << i)
#define PMW3610_WRITE_BIT BIT(7)

#define PMW3610_MOTION_MOT BIT(7) #1 : motion occured
#define PMW3610_MOTION_OVF BIT(4) #1 : motion overflow

#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

static uint8_t reg_read(uint8_t reg_address) {
    pinMode(SDIO, OUTPUT);
    for (int8_t i = 7; i >= 0; i--) {
        digitalWrite(SCLK, 0);
        digitalWrite(SDIO, reg_address & BIT(i));
        delayMicroseconds(1);
        digitalWrite(SCLK, 1);
        delayMicroseconds(1);
    }
    pinMode(SDIO, INPUT);
    delayMicroseconds(4);
    uint8_t r = 0;
    for (int8_t i = 7; i >= 0; i--) {
        digitalWrite(SCLK, 0);
        delayMicroseconds(1);
        digitalWrite(SCLK, 1);
        delayMicroseconds(1);
        r |= digitalRead(SDIO) << i;
    }
    delayMicroseconds(10);
    return r;
}

static void _reg_write(uint8_t reg_address, uint8_t data) {
    reg_address |= PMW3610_WRITE_BIT;

    pinMode(SDIO, OUTPUT);
    for (int8_t i = 7; i >= 0; i--) {
        digitalWrite(SCLK, 0);
        digitalWrite(SDIO, reg_address & BIT(i));
        delayMicroseconds(1);
        digitalWrite(SCLK, 1);
        delayMicroseconds(1);
    }
    for (int8_t i = 7; i >= 0; i--) {
        digitalWrite(SCLK, 0);
        digitalWrite(SDIO, data & BIT(i));
        delayMicroseconds(1);
        digitalWrite(SCLK, 1);
        delayMicroseconds(1);
    }
    delayMicroseconds(4);
}

static void reg_write(uint8_t reg_address, uint8_t data) {
    _reg_write(PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    _reg_write(reg_address, data);
    _reg_write(PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
}

// 200~3200, 1200 by default
static void set_cpi(uint32_t cpi) {
    uint8_t value = cpi / 200;
    reg_write(PMW3610_REG_SPI_PAGE0, 0xFF);
    reg_write(PMW3610_REG_RES_STEP, value);
    reg_write(PMW3610_REG_SPI_PAGE1, 0x00);
}

void setup() {
    pinMode(VBAT_ENABLE, OUTPUT);
    digitalWrite(VBAT_ENABLE, 0);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    pinMode(SCLK, OUTPUT);
    pinMode(SDIO, OUTPUT);
    pinMode(MOTION, INPUT);

    Serial.begin(38400);
    delay(5000);
    init_pmw3610();
}

void init_pmw3610() {
    reg_write(PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
    delay(50);

    reg_write(PMW3610_REG_OBSERVATION, 0x00);
    delay(10);
    uint8_t observation = reg_read(0x2D);
    if ((observation & 0x0F) != 0x0F) {
        Serial.println("Observation failure");
    } else {
        Serial.println("Observation OK");
    }

    reg_read(PMW3610_REG_MOTION);
    reg_read(PMW3610_REG_DELTA_X_L);
    reg_read(PMW3610_REG_DELTA_Y_L);
    reg_read(PMW3610_REG_DELTA_XY_H);

    // Read product ID multiple times to ensure sensor is stably working.
    for (int i = 0; i < 10; i++) {
        Serial.write("Product Id = ");
        Serial.println(reg_read(PMW3610_REG_PRODUCT_ID));
        Serial.write("Revision Id = ");
        Serial.println(reg_read(PMW3610_REG_REVISION_ID));
    }

    reg_write(PMW3610_REG_PERFORMANCE,
              0x0d);  // normal operation, 4ms, 4ms, 4ms
    // Time after which sensor goes from RUN to REST1 mode.
    reg_write(PMW3610_REG_RUN_DOWNSHIFT,
              0x04);  // 128ms (unit: pos-rate (4ms) * 8 = 32ms)
    reg_write(PMW3610_REG_REST1_PERIOD, 0x04);  // 40ms (unit: 10ms)
    reg_write(PMW3610_REG_REST1_DOWNSHIFT,
              0x0f);  // (unit: 16 * Rest1_sample_period (above, 40ms))

    set_cpi(200);
}

void loop() {
    if (Serial.available()) {
        String s = Serial.readString();
        if (s.startsWith("reset")) {
            init_pmw3610();
        }
    }
    if (digitalRead(MOTION) == 0) {
        uint8_t motion = reg_read(PMW3610_REG_MOTION);
        if (motion & BIT(7)) {
            uint16_t delta_x_l  = reg_read(PMW3610_REG_DELTA_X_L);
            uint16_t delta_y_l  = reg_read(PMW3610_REG_DELTA_Y_L);
            uint16_t delta_xy_h = reg_read(PMW3610_REG_DELTA_XY_H);
            // force-reset delta registers
            // reg_write(PMW3610_REG_MOTION, 0x09);
            int16_t delta_x =
                TOINT16(delta_x_l + ((delta_xy_h & 0xF0) << 4), 12);
            int16_t delta_y =
                TOINT16(delta_y_l + ((delta_xy_h & 0x0F) << 8), 12);
            Serial.print("M=");
            for (int i = 7; i >= 0; i--) {
                Serial.print((motion & BIT(i)) ? "1" : "0");
            }
            Serial.print(" dx_l =");
            for (int i = 7; i >= 0; i--) {
                Serial.print((delta_x_l & BIT(i)) ? "1" : "0");
            }
            Serial.print(" dy_l =");
            for (int i = 7; i >= 0; i--) {
                Serial.print((delta_y_l & BIT(i)) ? "1" : "0");
            }
            Serial.print(" dxy_h =");
            for (int i = 7; i >= 0; i--) {
                Serial.print((delta_xy_h & BIT(i)) ? "1" : "0");
            }
            Serial.printf(", x = %d y = %d\n", delta_x, delta_y);
        } else {
            Serial.print("M=");
            for (int i = 7; i >= 0; i--) {
                Serial.print((motion & BIT(i)) ? "1" : "0");
            }
            Serial.println();
        }
    }
}