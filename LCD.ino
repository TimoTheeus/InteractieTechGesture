
unsigned long lastPrint = 0;
unsigned long printDelay = 7000;
bool lcdCleared = true;
void setupLCD()
{
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  
}
void loopLCD() 
{
  unsigned long currentTime = millis();
  if(currentTime - lastPrint > printDelay&&!lcdCleared&&lastPrint!=0){
    lcd.clear();
    lcdCleared = true;
  }
}


void writeGoodJob(){
  lcdCleared = false;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Good job");
  lcd.setCursor(0,1);
  lcd.print("on washing");
  lastPrint = millis();
}
void writeWashYourHands()
{
  lcdCleared = false;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Wash your hands");
  lcd.setCursor(0,1);
  lcd.print("seriously");
  lastPrint = millis();
}
