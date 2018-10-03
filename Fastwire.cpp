// I2C library
//////////////////////
// Copyright(C) 2012
// Francesco Ferrara
// ferrara[at]libero[point]it
//////////////////////

#include "Fastwire.h"
#ifdef __AVR__
/*
FastWire
- 0.24 added stop
- 0.23 added reset

 This is a library to help faster programs to read I2C devices.
 Copyright(C) 2012 Francesco Ferrara
 occhiobello at gmail dot com
 [used by Jeff Rowberg for I2Cdevlib with permission]
 */

boolean Fastwire::waitInt() {
    int l = 250;
    while (!(TWCR & (1 << TWINT)) && l-- > 0);
    return l > 0;
}

void Fastwire::setup(int khz, boolean pullup) {
    TWCR = 0;
    #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
        // activate internal pull-ups for twi (PORTC bits 4 & 5)
        // as per note from atmega8 manual pg167
        if (pullup) PORTC |= ((1 << 4) | (1 << 5));
        else        PORTC &= ~((1 << 4) | (1 << 5));
    #elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
        // activate internal pull-ups for twi (PORTC bits 0 & 1)
        if (pullup) PORTC |= ((1 << 0) | (1 << 1));
        else        PORTC &= ~((1 << 0) | (1 << 1));
    #else
        // activate internal pull-ups for twi (PORTD bits 0 & 1)
        // as per note from atmega128 manual pg204
        if (pullup) PORTD |= ((1 << 0) | (1 << 1));
        else        PORTD &= ~((1 << 0) | (1 << 1));
    #endif

    TWSR = 0; // no prescaler => prescaler = 1
    TWBR = ((16000L / khz) - 16) / 2; // change the I2C clock rate
    TWCR = 1 << TWEN; // enable twi module, no interrupt
}

// added by Jeff Rowberg 2013-05-07:
// Arduino Wire-style "beginTransmission" function
// (takes 7-bit device address like the Wire method, NOT 8-bit: 0x68, not 0xD0/0xD1)
byte Fastwire::beginTransmission(byte device) {
    byte twst, retry;
    retry = 2;
    do {
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO) | (1 << TWSTA);
        if (!waitInt()) return 1;
        twst = TWSR & 0xF8;
        if (twst != TW_START && twst != TW_REP_START) return 2;

        //Serial.print(device, HEX);
        //Serial.print(" ");
        TWDR = device << 1; // send device address without read bit (1)
        TWCR = (1 << TWINT) | (1 << TWEN);
        if (!waitInt()) return 3;
        twst = TWSR & 0xF8;
    } while (twst == TW_MT_SLA_NACK && retry-- > 0);
    if (twst != TW_MT_SLA_ACK) return 4;
    return 0;
}

byte Fastwire::writeBuf(byte device, byte address, byte *data, byte num) {
    byte twst, retry;

    retry = 2;
    do {
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO) | (1 << TWSTA);
        if (!waitInt()) return 1;
        twst = TWSR & 0xF8;
        if (twst != TW_START && twst != TW_REP_START) return 2;

        //Serial.print(device, HEX);
        //Serial.print(" ");
        TWDR = device & 0xFE; // send device address without read bit (1)
        TWCR = (1 << TWINT) | (1 << TWEN);
        if (!waitInt()) return 3;
        twst = TWSR & 0xF8;
    } while (twst == TW_MT_SLA_NACK && retry-- > 0);
    if (twst != TW_MT_SLA_ACK) return 4;

    //Serial.print(address, HEX);
    //Serial.print(" ");
    TWDR = address; // send data to the previously addressed device
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!waitInt()) return 5;
    twst = TWSR & 0xF8;
    if (twst != TW_MT_DATA_ACK) return 6;

    for (byte i = 0; i < num; i++) {
        //Serial.print(data[i], HEX);
        //Serial.print(" ");
        TWDR = data[i]; // send data to the previously addressed device
        TWCR = (1 << TWINT) | (1 << TWEN);
        if (!waitInt()) return 7;
        twst = TWSR & 0xF8;
        if (twst != TW_MT_DATA_ACK) return 8;
    }
    //Serial.print("\n");

    return 0;
}

byte Fastwire::write(byte value) {
    byte twst;
    //Serial.println(value, HEX);
    TWDR = value; // send data
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!waitInt()) return 1;
    twst = TWSR & 0xF8;
    if (twst != TW_MT_DATA_ACK) return 2;
    return 0;
}

byte Fastwire::readBuf(byte device, byte address, byte *data, byte num) {
    byte twst, retry;

    retry = 200;
    do {
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO) | (1 << TWSTA);
        if (!waitInt()) return 16;
        twst = TWSR & 0xF8;
        if (twst != TW_START && twst != TW_REP_START) return 17;

        //Serial.print(device, HEX);
        //Serial.print(" ");
        TWDR = device & 0xfe; // send device address to write
        TWCR = (1 << TWINT) | (1 << TWEN);
        if (!waitInt()) return 18;
        twst = TWSR & 0xF8;
    } while (twst == TW_MT_SLA_NACK && retry-- > 0);
    Serial.println(twst, BIN);
    if (twst != TW_MT_SLA_ACK) return 19;

    //Serial.print(address, HEX);
    //Serial.print(" ");
    TWDR = address; // send data to the previously addressed device
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!waitInt()) return 20;
    twst = TWSR & 0xF8;
    if (twst != TW_MT_DATA_ACK) return 21;

    /***/

    retry = 2;
    do {
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO) | (1 << TWSTA);
        if (!waitInt()) return 22;
        twst = TWSR & 0xF8;
        if (twst != TW_START && twst != TW_REP_START) return 23;

        //Serial.print(device, HEX);
        //Serial.print(" ");
        TWDR = device | 0x01; // send device address with the read bit (1)
        TWCR = (1 << TWINT) | (1 << TWEN);
        if (!waitInt()) return 24;
        twst = TWSR & 0xF8;
    } while (twst == TW_MR_SLA_NACK && retry-- > 0);
    if (twst != TW_MR_SLA_ACK) return 25;

    for (uint8_t i = 0; i < num; i++) {
        if (i == num - 1)
            TWCR = (1 << TWINT) | (1 << TWEN);
        else
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
        if (!waitInt()) return 26;
        twst = TWSR & 0xF8;
        if (twst != TW_MR_DATA_ACK && twst != TW_MR_DATA_NACK) return twst;
        data[i] = TWDR;
        //Serial.print(data[i], HEX);
        //Serial.print(" ");
    }
    //Serial.print("\n");
    stop();

    return 0;
}

void Fastwire::reset() {
    TWCR = 0;
}

byte Fastwire::stop() {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    if (!waitInt()) return 1;
    return 0;
}

#endif