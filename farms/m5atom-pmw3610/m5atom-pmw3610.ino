// This code is based on https://github.com/inorichi/zmk-pmw3610-driver
#include <M5Atom.h>

// M5Atom pin assignment

#define SCK G22
#define SDIO G19
#define MOTION G23
#define NCS G33

// --

#define T_NCS_SCLK 1     /* 120 ns (rounded to 1us) */
#define T_SCLK_NCS_WR 10 /* 10 us */
#define T_SRAD 4         /* 4 us */
#define T_SRAD_MOTBR 4   /* same as T_SRAD */
#define T_SRX 1          /* 250 ns (rounded to 1 us) */
#define T_SWX 30         /* SWW: 30 us, SWR: 20 us */
#define T_BEXIT 1        /* 250 ns (rounded to 1us)*/

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
// #define BIT(i) (1 << i)
#define PMW3610_WRITE_BIT BIT(7)

CRGB dispColor(uint8_t g, uint8_t r, uint8_t b) {
    return (CRGB)((g << 16) | (r << 8) | b);
}

static void set_cs(bool cs) {
    if (!cs) {
        delayMicroseconds(T_NCS_SCLK);
    }
    digitalWrite(NCS, cs ? 0 : 1);
    if (cs) {
        delayMicroseconds(T_NCS_SCLK);
    }
}

static uint8_t reg_read(uint8_t reg_address) {
    set_cs(true);
    pinMode(SDIO, OUTPUT);
    for (int8_t i = 7; i >= 0; i--) {
        digitalWrite(SCK, 0);
        delayMicroseconds(1);
        digitalWrite(SDIO, reg_address & BIT(i));
        digitalWrite(SCK, 1);
        delayMicroseconds(5);
    }
    pinMode(SDIO, INPUT);
    delayMicroseconds(T_SRAD);
    delay(10);
    uint8_t r = 0;
    for (int8_t i = 7; i >= 0; i--) {
        digitalWrite(SCK, 0);
        delayMicroseconds(1);
        digitalWrite(SCK, 1);
        r |= digitalRead(SDIO) << i;
        delayMicroseconds(1);
    }
    set_cs(false);
    delayMicroseconds(T_SRX);
    return r;
}

static void _reg_write(uint8_t reg_address, uint8_t data) {
    reg_address |= PMW3610_WRITE_BIT;

    set_cs(true);
    pinMode(SDIO, OUTPUT);
    for (int8_t i = 7; i >= 0; i--) {
        digitalWrite(SCK, 0);
        digitalWrite(SDIO, reg_address & BIT(i));
        delayMicroseconds(1);
        digitalWrite(SCK, 1);
        delayMicroseconds(5);
    }
    for (int8_t i = 7; i >= 0; i--) {
        digitalWrite(SCK, 0);
        digitalWrite(SDIO, data & BIT(i));
        delayMicroseconds(1);
        digitalWrite(SCK, 1);
        delayMicroseconds(5);
    }
    delayMicroseconds(T_SCLK_NCS_WR);
    set_cs(false);
    delayMicroseconds(T_SWX);
}

static void reg_write(uint8_t reg_address, uint8_t data) {
    _reg_write(PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    _reg_write(reg_address, data);
    _reg_write(PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
}

struct MotionData {
    bool motion;
    int16_t dx;
    int16_t dy;
};

void setup() {
    M5.begin(true, false, true);
    M5.dis.drawpix(0, dispColor(0, 0, 0));
    pinMode(SCK, OUTPUT);
    pinMode(SDIO, OUTPUT);
    pinMode(NCS, OUTPUT);
    pinMode(MOTION, INPUT);

    // Serial.begin(38400);
    init_pmw3610();
}

void init_pmw3610() {
    set_cs(false);
    set_cs(true);
    reg_write(PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);

    delay(10);
    reg_write(PMW3610_REG_OBSERVATION, 0x00);

    delay(200);
    uint8_t obs = reg_read(PMW3610_REG_OBSERVATION);
    if ((obs & 0x0F) != 0x0F) {
        Serial.println("observation is wrong");
        return;
    }
    Serial.write("Product Id = ");
    Serial.println(reg_read(PMW3610_REG_PRODUCT_ID));
    Serial.write("Revision Id = ");
    Serial.println(reg_read(PMW3610_REG_REVISION_ID));

    delay(50);

    reg_read(PMW3610_REG_MOTION);
    reg_read(PMW3610_REG_DELTA_X_L);
    reg_read(PMW3610_REG_DELTA_Y_L);
    reg_read(PMW3610_REG_DELTA_XY_H);
}

void loop() {
    uint8_t motion = reg_read(PMW3610_REG_MOTION);
    if (motion & BIT(7)) {
        Serial.write("Motion = ");
        Serial.println(motion);
        Serial.write("delta_x = ");
        Serial.println(reg_read(PMW3610_REG_DELTA_X_L));
        Serial.write("delta_y = ");
        Serial.println(reg_read(PMW3610_REG_DELTA_Y_L));
        Serial.write("delta_xy = ");
        Serial.println(reg_read(PMW3610_REG_DELTA_XY_H));
    }
    delay(1000);
    M5.dis.drawpix(0, dispColor(255, 0, 0));
    delay(1000);
    M5.dis.drawpix(0, dispColor(0, 255, 0));
}