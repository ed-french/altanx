#include <Arduino.h>
#include <M5StickC.h>
#include <esp_now.h>
#include "WiFi.h"

#define ENABLE_BUZZING
#define ENABLE_LED

#define PIN_VIBRATION 26
#define VIBE_STOPPED LOW
#define VIBRATING HIGH

#define PIN_FRONT_BUTTON 37
#define PIN_SIDE_BUTTON 39
#define PRESSED false

#define PIN_LED 10

bool buzzing=false;
bool side_button_pressed=false;
bool front_button_pressed=false;

// Simple message to be passed back and forth
typedef struct struct_message {
  char text[32];
} struct_message;

struct_message message;

typedef struct
{
  bool is_master;
  bool is_synced;
  bool buzz_enabled;
  uint32_t time_offset;
} t_sync_state;

t_sync_state main_state{IS_MASTER,false,true,0};


// old_state stores the previous state so the display can be selectively updated
// it is initialised to be different in every regard so it can all be drawn the first time
t_sync_state old_state{!IS_MASTER,true,false,1};





// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status==ESP_NOW_SEND_SUCCESS)
  {
    main_state.time_offset=millis();
    main_state.is_synced=true;
  }
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

}


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  main_state.time_offset=millis();//Set synchronization
  main_state.is_synced=true;
  memcpy(&message, incomingData, sizeof(message));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("content ");
  Serial.println(message.text);
  
}



void M5_display_init()
{
  M5.begin();
  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(BLACK);
}

uint32_t M5_colour(uint8_t red,uint8_t green, uint8_t blue)
{
  return red << 11 | green << 5 | blue;
}

void M5_draw_master(bool is_master)
{
  int32_t width=is_master?90:112;
  int32_t top_left_x=80-(width/2);
  int32_t top_left_y=0;
  
  M5.Lcd.fillRect(top_left_x,top_left_y,width,22,BLACK);
  M5.Lcd.drawRect(top_left_x,top_left_y,width,22,is_master?GREEN:RED);
  M5.Lcd.setCursor(top_left_x+2,top_left_y+3);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(is_master?RED:GREEN);
  M5.Lcd.print(is_master?"Leading":"Following");

}

void M5_draw_sync(bool is_synced)
{
  int32_t top_left_x=35;
  int32_t top_left_y=50;
  M5.Lcd.fillRect(top_left_x,top_left_y,90,22,BLACK);
  M5.Lcd.drawRect(top_left_x,top_left_y,90,22,is_synced?GREEN:RED);
  M5.Lcd.setCursor(top_left_x+2,top_left_y+3);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(is_synced?GREEN:RED);
  M5.Lcd.print(is_synced?"Synced":"Waiting");
}



void update_display(t_sync_state main_state) 
{
  if (main_state.is_master!=old_state.is_master)
  {
    M5_draw_master(main_state.is_master);
  }
  if (main_state.is_synced!=old_state.is_synced)
  {
    M5_draw_sync(main_state.is_synced);
  }

  // M5.Lcd.fillRect(58,58,50,18,BLUE);
  // M5.Lcd.setCursor(60,60);
  // M5.Lcd.setTextSize(2);
  // M5.Lcd.setTextColor(left_side?YELLOW:MAGENTA);
  // M5.Lcd.print(buzzing?"Buzz":"Wait");

  old_state=main_state; // Stop it drawing stuff until it changes
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setCpuFrequencyMhz(80);// Slow down the cores to save a little juice
  
  pinMode(PIN_VIBRATION,OUTPUT);
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_VIBRATION,VIBE_STOPPED);
  pinMode(PIN_FRONT_BUTTON,INPUT);
  pinMode(PIN_SIDE_BUTTON,INPUT);
  delay(500);
  M5_display_init();
  update_display(main_state);

  Serial.print("This device MAC is : ");
  Serial.println(WiFi.macAddress());
  
  
  uint8_t pair_address[]=PAIR_MAC_ADDRESS;

  
  Serial.println("Paired in firmware with : ");
  for (uint8_t i=0;i<6;i++)
  {
    Serial.printf("%d=%x:",i,pair_address[i]);
  }
  Serial.println();

  Serial.printf("Device %s master?\n",main_state.is_master?"IS":"ISN'T");



    // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  #if IS_MASTER==true
  esp_now_register_send_cb(OnDataSent); // for master
  #else
  esp_now_register_recv_cb(OnDataRecv); // for slave
  #endif

 // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, pair_address, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

   
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  #if IS_MASTER==true
  // If we are the master, try and find the slave:

  while (!main_state.is_synced) // Will be changed by callback when the slave responds
  {
    Serial.println("Attempting to call to slave...");
    strcpy(message.text,"Hello slave!");
    esp_err_t result=esp_now_send(pair_address,(uint8_t *) &message,sizeof(message));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
    Serial.println("Error sending the data");
    }
    //Pause to give a chance for the slave to reply, allow reset if required(why?)
    for (uint8_t i=0;i<10;i++)
    {
      if (digitalRead(PIN_FRONT_BUTTON)==PRESSED)
      {
        esp_restart();
      }
      delay(100);
    }
    
  }

  #endif




}


void loop() {
  // put your main code here, to run repeatedly:
  uint32_t offset_now=((millis()-main_state.time_offset)/1000); // Alternate every second
  buzzing=main_state.buzz_enabled & main_state.is_synced & ((offset_now & 0x0001)^main_state.is_master); // Only buzz when synced
  
  #ifdef ENABLE_BUZZING
  digitalWrite(PIN_VIBRATION,buzzing?VIBRATING:VIBE_STOPPED);
  #endif
  #ifdef ENABLE_LED
  digitalWrite(PIN_LED,!buzzing);
  #endif
  update_display(main_state);
  side_button_pressed=digitalRead(PIN_SIDE_BUTTON)==PRESSED;
  front_button_pressed=digitalRead(PIN_FRONT_BUTTON)==PRESSED;
  // if (side_button_pressed)
  // {
  //   set_side(!left_side); // Toggle side
  //   delay(200); // Anti-bounce
  // }
  if (front_button_pressed)
  {
    // M5.Lcd.fillScreen(WHITE);
    // M5.Lcd.setTextColor(RED);
    // M5.Lcd.setCursor(0,0);
    // M5.Lcd.println("Release\nbuttons\ntogether");
    // delay(200); // Anti-bounce
    // while (digitalRead(PIN_FRONT_BUTTON)==PRESSED)
    // {
    //   M5.Lcd.fillRect(0,65,160,15,millis());
    // }
    // main_state.time_offset=millis();
    // set_side(left_side);
    // delay(200);// Anti-bounce
    esp_restart();
  }
  if (side_button_pressed)
  {
    main_state.buzz_enabled=!main_state.buzz_enabled;
    delay(200);//anti bounce
  }
  
  
  
  
  Serial.printf("offset_now: %d, buzzing: %d, side_button: %d, front_button %d\n",offset_now,buzzing,side_button_pressed,front_button_pressed);
  delay(200);
}