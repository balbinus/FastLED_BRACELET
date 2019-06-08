/**
 * FastLED_BRACELET
 * Copyright (C) 2018 balbinus
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <FastLED.h>
#if defined(ADAFRUIT_ITSYBITSY_M4_EXPRESS)
    // NeoPixel data pin
    #define DATA_PIN                    8
    // Interrupt pin from MPU6050
    #define ACCEL_INTERRUPT_PIN         2
#elif defined(ARDUINO_TRINKET_M0)
    // NeoPixel data pin
    #define DATA_PIN                    4
    // Interrupt pin from MPU6050
    #define ACCEL_INTERRUPT_PIN         1
#else
    #error NO DATA PIN
#endif
#define LED_TYPE            WS2812B
#define COLOR_ORDER         RGB

#define NUM_LEDS_RGB        11
#define NUM_LEDS_RGBW        8
#define NUM_LEDS            NUM_LEDS_RGBW

CRGB leds_rgb[NUM_LEDS_RGB];
CRGB leds[NUM_LEDS_RGBW];

#define BRIGHTNESS           32
#define FRAMES_PER_SECOND   120

#include <Adafruit_DotStar.h>

#include <Wire.h>
#include "MPU6050.h"

// Acceleration threshold (in milli-gs),
#define ACCEL_THRESHOLD             10
// and duration of said threshold (in milliseconds)
#define ACCEL_THRESHOLD_DURATION    50
// And what to do when one sample does not meet the threshold
// (0 = reset all count, 1 = decrement by 1, 2 = dec by 2, 3 = dec by 4)
#define ACCEL_THRESHOLD_NEG_DEC     0x02
// Release time of Motion Detected Event (for LED lighting, in seconds)
#define ACCEL_RELEASE               5
// Debug through serial?
#define SERIAL_DEBUG                0

// Rainbow as base instead of white breathing
#define PRIDE_EDITION               1

/** Current state **/
volatile struct {
    uint16_t hue             = 0;
    uint32_t motion_detected = 0;
    bool int_cleared         = true;
} gState;

void ISR_motion_detected()
{
    gState.motion_detected = millis();
    gState.int_cleared = false;
}

void setup()
{
    // Gyro and accelerometer self-test sensor output
    // + Bias corrections for gyro and accelerometer
    float selfTest[3], accelBias[3];
    
    Wire.begin();
#if SERIAL_DEBUG == 1
    Serial.begin(38400);
#endif

    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(ACCEL_INTERRUPT_PIN, INPUT);
    digitalWrite(ACCEL_INTERRUPT_PIN, LOW);
    
    // Builtin LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    // RGB strip
    FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds_rgb, NUM_LEDS_RGB).setCorrection(UncorrectedTemperature);
    
    // Turn off the APA102
    Adafruit_DotStar dotstar(1, 7, 8, DOTSTAR_BRG);
    dotstar.begin();
    dotstar.show();
    
#ifdef INIT_RANDOM_SEED_16
    // Initialize random seeds
    uint16_t rs = 0;
    for (uint8_t i = 0 ; i < 6 ; i++)
    {
        rs |= (analogRead(A3) & 0x3) << (i * 3);
        delay(25);
    }
    random16_set_seed(rs);
#endif

    // Signal start of configuration
    digitalWrite(LED_BUILTIN, HIGH);
    
    // If we could not detect an MPU6050, exit now
    // (don't even try to set it up, it'd be useless, but keep the red LED on)
    if (!MPU6050_is_connected())
    {
#if SERIAL_DEBUG == 1
        Serial.println("Could not connect to MPU6050 :(");
#endif
        return;
    }
#if SERIAL_DEBUG == 1
    Serial.println("MPU6050 is online...");
#endif

    // Start by performing self test and reporting values
    MPU6050_self_test(selfTest);
#if SERIAL_DEBUG == 1
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(selfTest[0], 1);
    Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(selfTest[1], 1);
    Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(selfTest[2], 1);
    Serial.println("% of factory value");

    if (selfTest[0] < 1.0f && selfTest[1] < 1.0f && selfTest[2] < 1.0f)
    {
        Serial.println("=> SELF TEST PASSED!");
    }
    else
    {
        Serial.println("=> SELF TEST FAILED :(");
    }
#endif

    // Calibrate gyro and accelerometers, load biases in bias registers
    MPU6050_calibrate(accelBias);
    MPU6050_init(AFS_2G, ACCEL_THRESHOLD, ACCEL_THRESHOLD_DURATION, ACCEL_THRESHOLD_NEG_DEC);
    attachInterrupt(digitalPinToInterrupt(ACCEL_INTERRUPT_PIN), ISR_motion_detected, RISING);
#if SERIAL_DEBUG == 1
    Serial.println("MPU6050 initialized for passive data mode.");       // Initialize device for passive mode read of acclerometer
#endif
    
    // Turn the LED off now.
    digitalWrite(LED_BUILTIN, LOW);
}

static inline void rgb_to_rgbw()
{
    uint32_t *lrgb32 = (uint32_t *) leds_rgb;
    for (uint8_t i = 0 ; i < NUM_LEDS_RGBW ; i++)
    {
        lrgb32[i] = (leds[i].b << 16) | (leds[i].r << 8) | (leds[i].g);
    }
}

static inline void rgb_all_white_to_rgbw(uint8_t scale)
{
    uint32_t *lrgb32 = (uint32_t *) leds_rgb;
    for (uint8_t i = 0 ; i < NUM_LEDS_RGBW ; i++)
    {
        lrgb32[i] = scale << 24;
    }
}

void loop()
{
    uint32_t now = millis();
    
    // FIXME: find something better than this hack to decide which animation to play
    
    // Motion detected: rainbow with white sparkle
    if (now > (ACCEL_RELEASE * 1000) && now - gState.motion_detected < (ACCEL_RELEASE * 1000))
    {
        if (!gState.int_cleared)
        {
            MPU6050_clear_interrupt();
            gState.int_cleared = true;
        }
        
        // Full luminosity
        FastLED.setBrightness(0xFF);
        
        // Rainbow base
        fill_rainbow(leds, NUM_LEDS, (gState.hue & 0xFF) << 2, 32);
        nscale8_video(leds, NUM_LEDS, 16);
        
        // Add white sparkle
        if ((random8() & 0x15) == 0x15)
        {
            uint8_t pos = random8(NUM_LEDS);
            leds[pos] = CRGB::White;
        }
        
        // Convert to RGBW
        rgb_to_rgbw();
    }
    // No motion: white breathing
    else
    {
#ifdef PRIDE_EDITION
        // Full luminosity
        FastLED.setBrightness(0xFF);

        // Rainbow base
        fill_rainbow(leds, NUM_LEDS, (gState.hue & 0xFF) << 2, 32);
        nscale8_video(leds, NUM_LEDS, 16);

        rgb_to_rgbw();
#else
        // Breathe white
        rgb_all_white_to_rgbw(0xFF);
        
        // Scale using setBrightness to allow temporal dithering to work
        // (https://github.com/FastLED/FastLED/wiki/FastLED-Temporal-Dithering)
        uint8_t tri_hue = gState.hue & 0x100 ? 0xFF - (gState.hue & 0xFF) : gState.hue & 0xFF;
        FastLED.setBrightness(scale8_video(BRIGHTNESS, ease8InOutCubic(tri_hue)));
#endif
    }

    // send the 'leds' array out to the actual LED strip
    FastLED.show();
    
    // insert a delay to keep the framerate modest
    FastLED.delay(1000/FRAMES_PER_SECOND);

    // do some periodic updates
    EVERY_N_MILLISECONDS(10)
    {
        // slowly cycle the "base color" through the rainbow
        gState.hue++;
        if (gState.hue > 0x1FF)
        {
            gState.hue = 0;
        }
    }
}
