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

#define DELAY_MS 1000 - 1

volatile byte rpmcount;

unsigned int rpm;

unsigned long timeold;

/*
 * Interrupt function which is called when the fan blade interrupts the
 * link in between the IR emitter and detector
 */
void isr0()
{
    //Each rotation, this interrupt function is run twice, so take that into consideration for
    //calculating RPM
    //Update count
    rpmcount++;
}

/*
 * Setup the arduino
 */
void setup()
{
    // Configure Serial.
    Serial.begin(115200);

    // Configure Digital Pins.
    pinMode(IRDET_PIN, INPUT_PULLUP);
    pinMode(MOTOR_PIN, OUTPUT);

    //The IR detector is connected to a digital pin, convert to interrupt pin
    //Triggers on FALLING (change from HIGH to LOW)
    attachInterrupt(digitalPinToInterrupt(IRDET_PIN), isr0, FALLING);

    //Turn the fan on
    analogWrite(MOTOR_PIN, 70);

    rpmcount = 0;
    rpm = 0;
    timeold = 0;
}

/*
 * Task loop
 */
void loop()
{
    //Update RPM every second
    delay(DELAY_MS);
    //Don't process interrupts during calculations
    detachInterrupt(0);
    //Note that this would be 60*1000/(millis() - timeold)*rpmcount if the interrupt
    //happened once per revolution instead of twice. Other multiples could be used
    //for multi-bladed propellers or fans
    rpm = 30*1000/(millis() - timeold)*rpmcount;
    timeold = millis();
    rpmcount = 0;

    //Write it out to serial port
    Serial.println(rpm,DEC);

    //Restart the interrupt processing
    attachInterrupt(0, isr0, FALLING);

}
