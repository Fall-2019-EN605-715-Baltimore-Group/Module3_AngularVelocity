/*
 * Project 3 Measure and Transmit Fan Speed and Thrust
 *
 * Original code from:
 * Instructables.com Arduino-BAsed Optical Tachometer
 * by CMPalmer on September 16, 2007
 *
 * Altered by:
 * Josh Lowe, Zach Richards, Nora Karsten
 * September 29, 2019
 *
 * Uses an IR LED and IR phototransistor to implement an optical tachometer.
 * The IR LED is connected to pin 13 and ran continually. A status LED is connected
 * to pin 12. Pin 2 (interrupt 0) is connected across the IR detector.
 *
 *
 */

#include <Arduino.h>
//#include <DataStore.h>

#define MOTOR_PIN   3   // Motor/fan is connected to digital pin 3
#define IRDET_PIN   2   // IR detector output connected to digital pin 2
#define IRLED_PIN   12

#define MOTOR_VAL_Q 55

#define SAMPLE_MS   1000

#define MIN_TO_SEC  60
#define SEC_TO_MSEC 1000

volatile unsigned intrCount = 0;

bool fCountMutex = false;

unsigned long prevSampleTime = 0;
unsigned totalRotations = 0;

char printTimeout = 10;

float angularVelocity = 0.0f;

/*
 * Primary Interrupt Function
 * Records number of times IR Emitter->Detector link is broken.
 */
void isr0() {  if (!fCountMutex) {(!digitalRead(IRDET_PIN))? ++intrCount : 0;} }

void setup()
{
    // Configure Serial.
    Serial.begin(115200);

    // Configure Digital Pins.
    pinMode(IRDET_PIN, INPUT);
    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(IRLED_PIN, OUTPUT);

    digitalWrite(IRLED_PIN, HIGH);

    //The IR detector is connected to a digital pin, convert to interrupt pin.
    //Triggers on FALLING (change from HIGH to LOW)
    attachInterrupt(digitalPinToInterrupt(IRDET_PIN), isr0, FALLING);

    //Turn the fan on
    analogWrite(MOTOR_PIN, MOTOR_VAL_Q);
}

void loop()
{
    fCountMutex = true;      // Take the mutex

    totalRotations = intrCount / 2;

    intrCount = 0;

    fCountMutex = false;    // Return the Mutex

    prevSampleTime = millis();

    angularVelocity = static_cast<float>(totalRotations)/*/(millis() - prevSampleTime)*/;

    //Write it out to serial port
    if (printTimeout++ % 2 == 0)
    {
        Serial.print(prevSampleTime); Serial.print(','); Serial.println(totalRotations);
    }

    delay(SAMPLE_MS);   // Wait for next sample
}
