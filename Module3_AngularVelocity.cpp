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
 * to pin 12. Pin 2 (interrupt 0) is connected across the IR detector. Motor is connected
 * to pin 3.
 *
 * This implementation uses a mutex to control access to shared data.
 * Whenever the interrupt routine wants to update is count, it must first check
 * the mutex state. If the mutex is available, it will update the count and exit.
 * The mutex is actuated by the main task loop. This processing path periodically checks
 * the interrupt counter, makes some calculations, prints the result, and then waits.
 * The critical section of this block is the reading of the counter. Whenever it wants
 * to do so, it must take the mutex, preventing the interrupt for modifying it, but also
 * omitting any interrupt events which occur during the time the mutex is taken.
 */

#include <Arduino.h>

#define MOTOR_PIN   3   // Motor/fan is connected to digital pin 3
#define IRDET_PIN   2   // IR detector output connected to digital pin 2
#define IRLED_PIN   12  // IR emitter connected to digital pin 12

#define MOTOR_VAL_Q 55  // Controls the speed of the motor. Usable range: (35,50)

#define SAMPLE_MS   1000    // Interrupt sampling rate.

#define PRINT_TIMEOUT 2     // Control print frequency.

volatile unsigned intrCount = 0;    // Number of interrtupts that have occured.

volatile bool fCountMutex = false;   // Mutex to control access to above count.

unsigned long prevSampleTime = 0;
unsigned totalRotations = 0;

char printTimeout = 0;

float angularVelocity = 0.0f;

/*
 * Primary Interrupt Function
 * Records number of times IR Emitter->Detector link is broken.
 * In an effort to suppress erronous interrupts, the interrupt pin is immediately
 * read to see if the link is still down, only counting the interrupt if the return
 * value is logic 0.
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

    // Active the LED.
    digitalWrite(IRLED_PIN, HIGH);

    //The IR detector is connected to a digital pin, convert to interrupt pin.
    //Triggers on FALLING (change from HIGH to LOW)
    attachInterrupt(digitalPinToInterrupt(IRDET_PIN), isr0, FALLING);

    //Turn the motor on
    analogWrite(MOTOR_PIN, MOTOR_VAL_Q);
}

void loop()
{
    fCountMutex = true;      // Take the mutex

    totalRotations = intrCount / 2; // Account for multiple interrupts per rotation.

    intrCount = 0;      // Reset the counter.

    fCountMutex = false;    // Return the Mutex

    prevSampleTime = millis();  // Save last record time.

    // angularVelocity would allow for modifications to the printed unit.
    // If desired, the appropriate time quanta could be substituted for the denominator of this equation.
    angularVelocity = static_cast<float>(totalRotations)/*/(millis() - prevSampleTime)*/;

    //Write it out to serial port
    // The current implementation displays:
    //  SAMPLE_TOA (msec) : FREQ (Hz)
    Serial.print(prevSampleTime); Serial.print(','); Serial.println(angularVelocity);

    delay(SAMPLE_MS);   // Wait for next sample
}
