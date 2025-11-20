
/****************************************************************
 *    TEAM 07: B. Smith and J. Zawatsky 
 *    CPEG222 Proj5, 11/21/25
 *    NucleoF466RE Arduino STM32F4xx 
 *    BME280 Environment Monitor
 ****************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_NeoPixel.h>
#include "stm32f4xx.h"

#ifndef SSD_ARRAY_H
#define SSD_ARRAY_H

#ifdef __cplusplus
extern "C" {
    #endif
    void SSD_init(void);
    void SSD_update(int digitSelect, int value, int decimalPoint);
    #ifdef __cplusplus
}
#endif
#endif

#define PIXEL_PIN   PA0 
#define NUMPIXELS   4 

#define BTN_PIN     PC13

#define RED pixels.Color(150, 0, 0)
#define PURPLE pixels.Color(150, 0, 150)
#define BLUE pixels.Color(0, 150, 150)
#define GREEN pixels.Color(0, 150, 0)

#define BAUDRATE 115200
#define BME_ADDRESS 0x76

#define SSD_RATE 200

#define MEASUREMENT_DELAY 20

Adafruit_BME280 bme; // I2C
Adafruit_NeoPixel pixels(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

HardwareTimer *timer2;

/**************** Variables ****************/

// State
typedef enum state {
    temp_C, temp_F, humidity, pressure
} state_e;
volatile state_e current_state;

// SSD State
volatile uint32_t digit_select;
volatile uint32_t decimal;

// Readings
volatile float tempC_reading;
volatile float tempF_reading;
volatile float humidity_reading;
volatile float pressure_reading;
volatile float SSD_reading;

/**************** Declarations ****************/

void TIM2_Handler(void);
void BTN_Handler(void);

void printValues(void);
void update_reading(void);
void update_SSD_reading(void);
void update_state(void);

/**************** Setup ****************/

void setup() {

    /***** Setup Serial Monitor *****/
    Serial.begin(BAUDRATE);
    while(!Serial);    // time to get serial running
    Serial.println(F("\n\e[4;37m\e[1;37mCPEG222 Project 5 - BME280 Values\e[0m\n"));

    /***** Setup BME Sensor *****/
    unsigned status;
    bme.begin(BME_ADDRESS);  

    /***** Setup Pixels *****/
    pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
    pixels.setBrightness(50);
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.fill(RED);
    pixels.show();

    /***** Setup SSD *****/
    SSD_init();

    /***** Setup BTN *****/
    pinMode(BTN_PIN, INPUT_PULLUP); // Set Button as input
    attachInterrupt(BTN_PIN, BTN_Handler, FALLING); // Button interrupt

    /***** Setup TIM2 *****/
    timer2 = new HardwareTimer(TIM2);
    timer2->setOverflow(SSD_RATE, HERTZ_FORMAT);
    timer2->attachInterrupt(TIM2_Handler);
    timer2->resume();

    /***** Initialize State Variables *****/
    current_state = temp_C;
    digit_select = 0;

    /***** Grab Initial Reading *****/
    update_reading();
}

/**************** Loop ****************/

void loop() { 
    printValues(); // Print to serial monitor
    for (int i=0; i < (5000/MEASUREMENT_DELAY); i++) {
        delay(MEASUREMENT_DELAY); // Grab new reading every 1 second ()
        update_reading();
    }
}

/**************** Definitions ****************/

/// @brief Prints bme280 readings to serial monitor
void printValues() {
    Serial.printf("\e[0;37mTemperature = \033[31m");
    Serial.print(bme.readTemperature());
    Serial.print(" °C ");

    Serial.printf("\e[0;37m/ \033[35m");
    Serial.print((bme.readTemperature() * (9.0f/5.0f)) + 32.0f); // Convert C to F
    Serial.print(" °F\t");

    Serial.printf("\e[0;37mRelHum = \033[36m");
    Serial.print(bme.readHumidity());
    Serial.print(" %\t");

    Serial.printf("\e[0;37mPressure = \033[32m");
    Serial.print((bme.readPressure() / 100.0f) * 0.00098692f, 4); // convert hPa to atm
    Serial.println(" atm");

    Serial.println();
}

/// @brief Updates current state by simply moving to the next state sequentially;
///        Also updates neopixel color displayed
void update_state(void) {
    pixels.clear();
    switch (current_state) {
        case temp_C:
            current_state = temp_F;
            pixels.fill(PURPLE);
            break;
        case temp_F:
            current_state = humidity;
            pixels.fill(BLUE);
            break;
        case humidity:
            current_state = pressure;
            pixels.fill(GREEN);
            break;
        case pressure:
            current_state = temp_C;
            pixels.fill(RED);
            break;
        default:
            break;
    }
    pixels.show();
}

/// @brief Takes new readings of bme280
void update_reading(void) {
    tempC_reading = bme.readTemperature();
    tempF_reading = tempC_reading * (9.0f/5.0f) + 32.0f; // Convert C to F
    humidity_reading = bme.readHumidity();
    pressure_reading = (bme.readPressure() / 100.0f) * 0.00098692f; // convert hPa to atm
}

void update_SSD_reading(void) {
    switch (current_state) {
        case temp_C:
            SSD_reading = tempC_reading * 100;
            break;
        case temp_F:
            SSD_reading = tempF_reading * 100;
            break;
        case humidity:
            SSD_reading = humidity_reading * 100;
            break;
        case pressure:
            SSD_reading = pressure_reading * 1000;
            break;
        default:
            break;
    }
}

/// @brief Updates SSD with most recent reading, and according to current state 200 times a second
void TIM2_Handler(void){

    digit_select = (digit_select + 1) % 4;

    update_SSD_reading();

    if (current_state == pressure) decimal = 1;
    else decimal = 2;

    SSD_update(digit_select, SSD_reading, decimal);
}

/// @brief Handles button presses to update the what measurement to display (state)
void BTN_Handler(void) {
    update_state();
}
