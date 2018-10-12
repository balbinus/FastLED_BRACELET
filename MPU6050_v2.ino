
/**
 * Based off MPU6050 Basic Example Code by: Kris Winer (date: May 1, 2014)
 * https://github.com/kriswiner/MPU6050
 * License: Beerware - Use this code however you'd like. If you find it useful
 * you can buy me a beer some time.
 */

#include <FastLED.h>
#if defined(ADAFRUIT_ITSYBITSY_M4_EXPRESS)
    #define DATA_PIN        8
    #define CLK_PIN         6
    // Interrupt pin from MPU6050
    #define ACCEL_INTERRUPT_PIN         2
#elif defined(ARDUINO_TRINKET_M0)
    #define DATA_PIN        7
    #define CLK_PIN         8
    // Interrupt pin from MPU6050
    #define ACCEL_INTERRUPT_PIN         1
#else
    #error NO DATA PIN
#endif
#define LED_TYPE            APA102
#define COLOR_ORDER         GRB
#define NUM_LEDS            1
#define LED1                0
#define LED2                1

#define BRIGHTNESS          0xFF
#define FRAMES_PER_SECOND   120

CRGB leds[NUM_LEDS];

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

/** Current state **/
volatile struct {
    uint8_t hue              = 0;
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
    float selfTest[3],                                                         // Gyro and accelerometer self-test sensor output
          accelBias[3];                                                        // Bias corrections for gyro and accelerometer
    
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
    
    // DotStar
    FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS);
    
    // set master brightness control
    FastLED.setBrightness(BRIGHTNESS);

    if (MPU6050_is_connected())
    {
        digitalWrite(LED_BUILTIN, HIGH);
#if SERIAL_DEBUG == 1
        Serial.println("MPU6050 is online...");
#endif

        MPU6050_self_test(selfTest);                                           // Start by performing self test and reporting values
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

        MPU6050_calibrate(accelBias);                                          // Calibrate gyro and accelerometers, load biases in bias registers
        MPU6050_init(AFS_2G, ACCEL_THRESHOLD, ACCEL_THRESHOLD_DURATION, ACCEL_THRESHOLD_NEG_DEC);
        attachInterrupt(digitalPinToInterrupt(ACCEL_INTERRUPT_PIN), ISR_motion_detected, RISING);
#if SERIAL_DEBUG == 1
        Serial.println("MPU6050 initialized for passive data mode....");       // Initialize device for passive mode read of acclerometer
#endif
        digitalWrite(LED_BUILTIN, LOW);
    }
    else
    {
#if SERIAL_DEBUG == 1
        Serial.print("Could not connect to MPU6050: 0x");
        Serial.println(c, HEX);
#endif
        while (1)                                                              // Loop forever if communication doesn't happen
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(500);
            digitalWrite(LED_BUILTIN, LOW);
            delay(500);
        }
    }
}

void loop()
{
    uint32_t now = millis();
    // FIXME: find something better than this hack
    if (now > (ACCEL_RELEASE * 1000) && now - gState.motion_detected < (ACCEL_RELEASE * 1000))
    {
        if (!gState.int_cleared)
        {
            MPU6050_clear_interrupt();
            gState.int_cleared = true;
        }
        
        // Rainbow
        leds[LED1] = CHSV(gState.hue, 0xFF, 0xFF);
    }
    else
    {
        // Breathe white
        leds[LED1] = CRGB::White;
        leds[LED1].nscale8_video(cubicwave8(gState.hue));
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
    }
}

