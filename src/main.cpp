#include <Arduino.h>
#include <M5StickC.h>

#define ENABLE_BUZZING

#define PIN_VIBRATION 26
#define VIBE_STOPPED LOW
#define VIBRATING HIGH

#define PIN_FRONT_BUTTON 37
#define PIN_SIDE_BUTTON 39
#define PRESSED false

#define PIN_LED 10

bool left_side=true; // Flag to indicate if it's the left or right module
bool buzzing=false;
bool side_button_pressed=false;
bool front_button_pressed=false;

uint32_t time_offset=0;

void set_side(bool new_side_is_left)
{
  left_side=new_side_is_left;
  // Write the new side to the display
  M5.Lcd.fillScreen(left_side?RED:GREEN); // Port color
  M5.Lcd.setCursor(left_side?30:25,20);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(left_side?YELLOW:MAGENTA);
  M5.Lcd.print(left_side?"Left Side":"Right Side");

}


void update_display()
{
  M5.Lcd.fillRect(58,58,50,18,BLUE);
  M5.Lcd.setCursor(60,60);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(left_side?YELLOW:MAGENTA);
  M5.Lcd.print(buzzing?"Buzz":"Wait");

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(PIN_VIBRATION,OUTPUT);
  digitalWrite(PIN_VIBRATION,VIBE_STOPPED);
  pinMode(PIN_FRONT_BUTTON,INPUT);
  pinMode(PIN_SIDE_BUTTON,INPUT);
  delay(500);
  M5.begin();
  M5.Lcd.setRotation(1);
  set_side(left_side);

}


void loop() {
  // put your main code here, to run repeatedly:
  uint32_t offset_now=((millis()-time_offset)/1000); // Alternate every second
  buzzing=(offset_now & 0x0001)^left_side;
  
  #ifdef ENABLE_BUZZING
  digitalWrite(PIN_VIBRATION,buzzing?VIBRATING:VIBE_STOPPED);
  #endif
  digitalWrite(PIN_LED,buzzing);
  update_display();
  side_button_pressed=digitalRead(PIN_SIDE_BUTTON)==PRESSED;
  front_button_pressed=digitalRead(PIN_FRONT_BUTTON)==PRESSED;
  if (side_button_pressed)
  {
    set_side(!left_side); // Toggle side
    delay(200); // Anti-bounce
  }
  if (front_button_pressed)
  {
    M5.Lcd.fillScreen(WHITE);
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setCursor(0,0);
    M5.Lcd.println("Release\nbuttons\ntogether");
    delay(200); // Anti-bounce
    while (digitalRead(PIN_FRONT_BUTTON)==PRESSED)
    {
      M5.Lcd.fillRect(0,65,160,15,millis());
    }
    time_offset=millis();
    set_side(left_side);
    delay(200);// Anti-bounce
  }
  
  
  
  
  Serial.printf("offset_now: %d, buzzing: %d, side_button: %d, front_button %d\n",offset_now,buzzing,side_button_pressed,front_button_pressed);
  delay(200);
}