#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//LCD SCREEN ------------------------------------------
// include the library code:
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 7, 5, 4, 3, 2);

bool hasWashedHands = false;
bool isWashing = false;
//for serial communication
int incomingByte=0;
//timer variables ---------------------------------------
unsigned long lastSprayTime=0;
unsigned long timeBetweenSprays = 10000;
unsigned long washingStartTime = 0;
unsigned long washingCutoff = 1500;
// ------------------------------------------------------

// set pin numbers:
const int sprayPin =  12;      // spray pin

void setup()
{
  Serial.begin(9600);
  
  // initialize the LED pin as an output:
  pinMode(sprayPin, OUTPUT);
  setupLCD();
}

void loop()
{
  loopLCD();
   unsigned long currentTime = millis();
     if(currentTime - washingStartTime > washingCutoff&&isWashing){
      hasWashedHands = true;
    }
  if(Serial.available()>0){
    //read incoming message
    incomingByte = Serial.read();
    //Spray once
    if(incomingByte == '1'&& currentTime - lastSprayTime > timeBetweenSprays){
    checkWashed();
    sprayOnce();
    }
    //Spray twice
    else if(incomingByte == '2'&& currentTime - lastSprayTime > timeBetweenSprays){
    checkWashed();
    sprayTwice();
    }
    else if(incomingByte == 'w'){
      if(!isWashing){
        washingStartTime = millis();
        isWashing = true;
      }
    }
    else{isWashing = false;}
  }
}
void checkWashed(){
  if(hasWashedHands){
    writeGoodJob();
    hasWashedHands = false;
  }
  else{
    writeWashYourHands();
  }
}
void sprayTwice()
{
  lastSprayTime = millis();
  sprayOnce();
  delay(2000);
  sprayOnce();
}

void sprayOnce()
{
  lastSprayTime = millis();
  digitalWrite(sprayPin, HIGH);
  delay(1000);
  digitalWrite(sprayPin,LOW);
}


