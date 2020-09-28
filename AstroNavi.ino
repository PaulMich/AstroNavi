#include "DHT.h"
#define DHT11_PIN 2
DHT dht;

int HumidityLastValue = 0;
int TemperatureLastValue = 0;

int HumidityCurrentValue = 0;
int TemperatureCurrentValue = 0;

// Used for generating interrupts using CLK signal
const int AzimuthPinA = 19;
const int AltitudePinA = 18;

// Used for reading DT signal
const int AzimuthPinB = 7;
const int AltitudePinB = 6;

// Keep track of last rotary value
int AzimuthLastCount = 0;
int AltitudeLastCount = 50;

// Updated by the ISR (Interrupt Service Routine)
volatile int AzimuthVirtualPosition = 0;
volatile int AltitudeVirtualPosition = 50;

// ------------------------------------------------------------------
// INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT
// ------------------------------------------------------------------
void isrAz ()  {
  static unsigned long AzLastInterruptTime = 0;
  unsigned long AzInterruptTime = millis();

  // If interrupts come faster than 5ms, assume it's a bounce and ignore
  if (AzInterruptTime - AzLastInterruptTime > 1) {
    if (digitalRead(AzimuthPinB) == LOW)
    {
      AzimuthVirtualPosition-- ; // Could be -5 or -10
    }
    else {
      AzimuthVirtualPosition++ ; // Could be +5 or +10
    }
  }
  // Keep track of when we were here last (no more than every 5ms)
  AzLastInterruptTime = AzInterruptTime;
}

void isrAl ()  {
  static unsigned long AlLastInterruptTime = 0;
  unsigned long AlInterruptTime = millis();

  // If interrupts come faster than 5ms, assume it's a bounce and ignore
  if (AlInterruptTime - AlLastInterruptTime > 1) {
    if (digitalRead(AltitudePinB) == LOW)
    {
      AltitudeVirtualPosition-- ; // Could be -5 or -10
    }
    else {
      AltitudeVirtualPosition++ ; // Could be +5 or +10
    }
  }
  // Keep track of when we were here last (no more than every 5ms)
  AlLastInterruptTime = AlInterruptTime;
}
 
#include <Wire.h>                  // Include Wire library (required for I2C devices)
#include <LiquidCrystal_I2C.h>     // Include LiquidCrystal_I2C library 
 
LiquidCrystal_I2C lcd(0x20, 16, 2);  // Configure LiquidCrystal_I2C library with 0x27 address, 16 columns and 2 rows
 
void setup() {
  dht.setup(DHT11_PIN);
  
  // Rotary pulses are INPUTs
  pinMode(AzimuthPinA, INPUT);
  pinMode(AzimuthPinB, INPUT);
  pinMode(AltitudePinA, INPUT);
  pinMode(AltitudePinB, INPUT);
  
  lcd.init();                        // Initialize I2C LCD module
 
  lcd.noBacklight();                   // Turn backlight ON

  lcd.setCursor(0, 0);               // Go to column 0, row 0
  lcd.print("ASTROnavi");
  lcd.setCursor(0, 1);               // Go to column 0, row 1
  lcd.print("PaulMich 09.2020");
 
  delay(2000);
  lcd.clear();
  
  // Attach the routine to service the interrupts
  attachInterrupt(digitalPinToInterrupt(AzimuthPinA), isrAz, LOW);
  attachInterrupt(digitalPinToInterrupt(AltitudePinA), isrAl, LOW);
  
  lcd.setCursor(0, 0);             
  lcd.print("Az:  ");
  lcd.setCursor(0, 1);
  lcd.print("Alt: ");
}
 
void loop() {
  HumidityCurrentValue = dht.getHumidity();
  TemperatureCurrentValue = dht.getTemperature();

  if(HumidityCurrentValue != HumidityLastValue) {
    lcd.setCursor(11, 0);
    lcd.print(HumidityCurrentValue);
    lcd.setCursor(13, 0);
    lcd.print("%RH");

    HumidityLastValue = HumidityCurrentValue;
  }

  if(TemperatureCurrentValue != TemperatureLastValue) {
    lcd.setCursor(11, 1);
    lcd.print(TemperatureCurrentValue);
    lcd.setCursor(14, 1);
    lcd.print("*C");

    TemperatureLastValue = TemperatureCurrentValue;
  }

  // Restrict value from 0 to +360
  if(AzimuthVirtualPosition > 360) AzimuthVirtualPosition = 0;
  else if(AzimuthVirtualPosition < 0) AzimuthVirtualPosition = 360;

  // Restrict value from 0 to +90
  if(AltitudeVirtualPosition > 90) AltitudeVirtualPosition = 0;
  else if(AltitudeVirtualPosition < 0) AltitudeVirtualPosition = 90;
  
  // If the current rotary switch position has changed then update everything
  if (AzimuthVirtualPosition != AzimuthLastCount) {

    // Write out to LCD the value and direction
    lcd.setCursor(0, 0);               // Go to column 0, row 0
    lcd.print("Az:  ");
    
    if(AzimuthVirtualPosition < 10) lcd.print("  ");
    else if(AzimuthVirtualPosition >= 10 && AzimuthVirtualPosition < 100) lcd.print(" ");
    
    lcd.print(AzimuthVirtualPosition);
    
    // Keep track of this new value
    AzimuthLastCount = AzimuthVirtualPosition ;
  }

  if (AltitudeVirtualPosition != AltitudeLastCount) {

    // Write out to LCD the value and direction
    lcd.setCursor(0, 1);               // Go to column 0, row 0
    lcd.print("Alt: ");

    if(AltitudeVirtualPosition < 10) lcd.print("  ");
    else if(AltitudeVirtualPosition >= 10 && AltitudeVirtualPosition < 100) lcd.print(" ");
    
    lcd.print(AltitudeVirtualPosition);

    // Keep track of this new value
    AltitudeLastCount = AltitudeVirtualPosition ;
  }
}
