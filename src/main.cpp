// (c) Ed French 2021



/*
          Bilateral alternating stimulation
          =================================


Signalling methodology
======================


 State    Leader                                    Follower
 =====    ======                                    ========

 PAIRING  1. Leader broadcasts pairing avail.
                                                    2. Follower Receives broadcast
                                                    notes leader's address

                                                    3. Follower sends Echo message directly
                                                    sets time offset
                                                    Follower now synced and paired
          4. Leader records follower mac
          sets time offset
          Leader now synced and paired              

    -------------------------------------------------------------------

  SYNCING  1. Leader sends sync message to
              follower mac                          2. Follower receives targeted sync message
                                                    3. Follower sends echo message directly
                                                    sets time offset
                                                    Follower now synced
           4. Leader sets time offset
           Leader now synced






*/



#include <Arduino.h>
#include <Preferences.h>


#define SAVE_PEER_INFO

#ifdef BOARD_TYPE_M5STICKC

// M5 specific libraries
    #include <M5StickC.h>

#endif

#ifdef BOARD_TYPE_TDISPLAY
  #include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
  #include <SPI.h>
  
  TFT_eSPI tft = TFT_eSPI(135,240);  // Invoke library, pins defined in User_Setup.h

  #define TFT_BLACK 0x0000 // black
#endif


#include <esp_now.h>
#include "WiFi.h"



#define BUZZ_PERIOD_MS 2000 // Complete period including on and off
#define LEADER_BUZZ_START_MS 0 //
#define LEADER_BUZZ_END_MS 900 // Allow leader to finish a bit early so no overlap
#define FOLLOWER_BUZZ_START_MS 1000
#define FOLLOWER_BUZZ_END_MS 1900

#define PWM_CHANNEL 0
#define PWM_FREQ 4000
#define PWM_RESOLUTION 8

#define PWM_LEVEL 128

#define SHORT_BUTTON_THRESHOLD 3000
#define VERY_LONG_BUTTON_THRESHOLD 12000

#define LOOP_DELAY_MS 257 // Make much longer when debugging as it's easier to follow serial messages


/*

The following are enabled in the platformio.ini depending on the boards concerned

#define ENABLE_BUZZING
#define ENABLE_DISPLAY
#define ENABLE_LED

*/
// Pin definitions
#ifdef BOARD_TYPE_TDISPLAY
  #define PIN_VIBRATION 27
  #define PIN_FRONT_BUTTON 35 
  #define WAKE_UP_PIN_DEFN GPIO_NUM_35
#endif
#ifdef BOARD_TYPE_M5STICKC
  #define PIN_VIBRATION 26
  #define PIN_FRONT_BUTTON 37
  #define PIN_SIDE_BUTTON 39
#endif

#define VIBE_STOPPED LOW
#define VIBRATING HIGH

#define WIFI_CHANNEL 0

#define PRESSED false

#define PIN_LED 10

const char * pair_message_text="Altanx pair requested";
const char * sync_message_text="Altanx sync requested";
const char * follower_echo_pair_text="Altanx follower echoing pair";
const char * follower_echo_sync_text="Altanx follower echoing sync";

uint8_t broadcast_addr[]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t blank_partner[]={0,0,0,0,0,0};

int16_t phase_ms; // How many ms through the phase at the start of the main loop

Preferences preferences;

esp_now_peer_info_t peerInfo;

bool buzzing=false;
bool radio_on=false;

void delay_with_yield(uint32_t ms)
{
  yield();
  if (ms<100)
  {
    delay(ms);
  } else {
    uint32_t remaining=ms;
    while (true)
    {
      if (remaining<100)
      {
        delay(remaining);
        break;
      } else {
        delay(50);
        yield();
        remaining-=50;
      }
    }
  }
}

struct button_state
{
  bool pressed;
  uint16_t press_length_ms;
};

button_state side_button={false,0};
button_state front_button={false,0};


// Simple message to be passed back and forth
typedef struct struct_message {
  char text[32];
} struct_message;

typedef struct received_msg {
  struct_message message;
  uint8_t mac_addr[6];
  bool new_ready=false;
};

received_msg last_received;

enum pairing_states 
{
  BLANK_WAITING_TO_START_PAIRING=0,
  PAIRING=1,
  PAIRED_NOT_SYNCED=2,
  SYNCING=3,
  PAIRED_SYNCED=4,
  DUMMY=5
};

static const char *state_names[] =
        { "blank", "Pairing", "paired not synced", "syncing","paired+sync","dummy" };



struct_message message;

typedef struct
{
  enum pairing_states pairing_state;
  uint8_t partner[6];
  bool is_leader;
  bool is_synced;
  bool buzz_enabled;
  bool led_enabled;
  uint32_t time_offset;
  uint32_t state_change_time;
} t_sync_state;

#ifdef IS_LEADER
  bool is_leader_def=true;
#else
  bool is_leader_def=false;
#endif

#ifdef SAVE_PEER_INFO
  bool saving_peer_info=true;
#else
  bool saving_peer_info=false;
#endif


bool leader_received_echo=false;

t_sync_state main_state={BLANK_WAITING_TO_START_PAIRING, \
                        {0,0,0,0,0,0}, \
                        is_leader_def, \
                        false, \
                        true, \
                        false, \
                        0, \
                        0}; // Will be overwritten from preferences


// old_state stores the previous state so the display can be selectively updated
// it is initialised to be different in every regard so it can all be drawn the first time
t_sync_state old_state={DUMMY, \
                        {0,0,0,0,0,0},\
                        !is_leader_def, \
                        true, \
                        false, \
                        false, \
                        1, \
                        1};


void buff_print_mac(char * buffer,uint8_t * mac_addr)
{
  // Writes a nicely formatted mac address
  sprintf(buffer,"%x:%x:%x:%x:%x:%x",mac_addr[0],mac_addr[1],mac_addr[2],mac_addr[3],mac_addr[4],mac_addr[5]);
}


void OnFollowerSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  char buff[40];
  buff_print_mac(buff,(uint8_t*)mac_addr);
  Serial.printf("Mac address: %s\n",buff);
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


// callback when data is sent
void OnLeaderSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // if (status==ESP_NOW_SEND_SUCCESS && memcmp(mac_addr,broadcast_addr,6)!=0)
  // {
  //   main_state.time_offset=millis();
  //   main_state.is_synced=true;
  // }
  Serial.print("\r\nLast Packet Send Status:\t");
  char buff[40];
  buff_print_mac(buff,(uint8_t*)mac_addr);
  Serial.printf("Mac address: %s\n",buff);
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void change_pairing_state(pairing_states new_state, const char * marker)
{
  Serial.printf("\n=============================\n"
                "Changing from : %s --to--> %s\n"
                "At marker: %s\n"
                "===============================\n", \
                state_names[main_state.pairing_state], \
                state_names[new_state], \
                marker);
  main_state.pairing_state=new_state; 
  main_state.state_change_time=millis();
}


void update_display(t_sync_state main_state,bool force_update=false) 
{
  // if (main_state.is_leader!=old_state.is_leader)
  // {
  //   #ifdef BOARD_TYPE_M5STICKC
  //   M5_draw_leader(main_state.is_leader);
  //   #endif
  // }
  // if (main_state.is_synced!=old_state.is_synced)
  // {
  //   #ifdef BOARD_TYPE_M5STICKC
  //   M5_draw_sync(main_state.is_synced);
  //   #endif
  // }
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(2);
  tft.println(main_state.is_leader?"leader":"follower");
  tft.println(main_state.is_synced?"synced":"unsynced");
  tft.printf("Pair state: %d\n",main_state.pairing_state);
  //tft.printf("PartnrLS: %hhx\n",main_state.partner[5]);
  char temp_buffer[30];
  buff_print_mac(temp_buffer,main_state.partner);
  tft.println(temp_buffer);
  tft.println(state_names[main_state.pairing_state]);
  tft.printf("Radio: %s",radio_on?"on":"off");

  


}

void show_message(uint8_t seconds,const char* message)
{
  
  #ifdef ENABLE_DISPLAY
  Serial.printf("About to show: %s\n",message);
  delay(300);
  char lines[3][30];
  for (uint8_t i=0;i<3;i++)
  {
    for (uint8_t j=0;j<30;j++)
    {
      lines[i][j]=0;
    }
  }
  uint16_t ptr=0;
  uint8_t lineno=0;
  uint8_t line_char_count=0;
  while(true)
  {
    if (message[ptr]==0) break; // End of input
    if (message[ptr]==13 || message[ptr]==10)
    {
      lineno++;
      line_char_count=0;
      if (lineno>3) break;
    } else {
      if (line_char_count<20) // ignore overflow
      {
        lines[lineno][line_char_count]=message[ptr];
        //Serial.printf("Added: %c\n",message[ptr]);
      }
      line_char_count++;
      if (line_char_count>30)
      {
        Serial.println("String too long, ignoring");
        break;
      }
    }
    if (lineno>3) break;
    ptr++;
  }
  for (uint8_t y=0;y<lineno+1;y++)
  {
    Serial.printf("Line %d: %s\n",y,lines[y]);
  }
  

  tft.fillScreen(TFT_RED);
  tft.drawRect(5,5,230,125,TFT_WHITE);
  
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  Serial.println("Starting to write to screen....");
  for (uint8_t y=0;y<lineno+1;y++)
  {
    tft.setCursor(10,10+14*y); 
    if (strlen(lines[y])>20)
    {
      Serial.println("Ignoring too long string");
      delay_with_yield(1000);
    } else {
      Serial.println(lines[y]);
      tft.print(lines[y]);
    }
    
    
  }
  Serial.println("Written to screen");

  

  delay_with_yield(seconds*1000);
  Serial.println("Completed delay");
  update_display(main_state,true);
  Serial.println("Display update done");

  #endif
}

void switch_off_wifi()
{
  Serial.println("Turning radio off...");
  delay_with_yield(1000);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  radio_on=false;
  Serial.println("Radio now off");
}


void save_state()
{
  // Don't save Pairing or syncing or synced modes- but copy first
  // so we don't change the live state...

  t_sync_state temp_state;
  memcpy(&temp_state,&main_state,sizeof(main_state));
  if (temp_state.pairing_state==PAIRING)
  {
    temp_state.pairing_state=BLANK_WAITING_TO_START_PAIRING;
  }
  if (temp_state.pairing_state==SYNCING)
  {
    temp_state.pairing_state=PAIRED_NOT_SYNCED;
  }
  if (temp_state.pairing_state==PAIRED_SYNCED)
  {
    temp_state.pairing_state=PAIRED_NOT_SYNCED;
  }
  temp_state.is_synced=false; // save it without is_synced set
  preferences.putBytes("syststate",&temp_state,sizeof(temp_state));
}

void leader_pairing_rx(received_msg rx)
{
   if (strcmp(rx.message.text,follower_echo_pair_text)!=0)
    {
      const char * buffer="\n\n==================\n"
                          "ERROR - expected pairing message\n"
                          "Ignoring!";
      Serial.println(buffer);
      show_message(5,"ERROR!\nTry re-pair");
      return;
    }
    // Valid pairing message so pair!

    // Note valid follower address
    memcpy(main_state.partner,rx.mac_addr,6);
    main_state.time_offset=millis();//Set synchronization
    main_state.is_synced=true;
    change_pairing_state(PAIRED_SYNCED,"Successful pair");
    switch_off_wifi();
    show_message(3,"Paired\nOK"); 
    save_state();
    rx.new_ready=false; // Flag it's now processed and we can rx another
}

void leader_syncing_rx(received_msg rx)
{
// Checks:
    if (strcmp(rx.message.text,follower_echo_sync_text)!=0 || \
        memcmp(rx.mac_addr,main_state.partner,6)!=0)
    {
      const char * buffer="\n\n==================\n"
                          "ERROR - expected sync message\n"
                          "Ignoring!";
      Serial.println(buffer);
      show_message(5,"ERROR!\nTry switch off");
      char tempbuff[20];
      buff_print_mac(tempbuff,rx.mac_addr);
      Serial.printf("Incoming message from mac: %s\n",tempbuff);
      Serial.printf("Message content: %s\n",rx.message.text);
      rx.new_ready=false;
      return;
    }
    // Valid sync message received
    main_state.time_offset=millis();//Set synchronization
    main_state.is_synced=true;
    change_pairing_state(PAIRED_SYNCED,"Successful sync");
    switch_off_wifi();
    rx.new_ready=false; // Flag it's now processed and we can rx another
}

void follower_pairing_rx(received_msg rx)
{
   // Checks
    if (strcmp(rx.message.text,pair_message_text)!=0)
    {
      const char * buffer="\n\n==================\n"
                          "ERROR - expected pairing message\n"
                          "Ignoring!";
      Serial.println(buffer);
      show_message(5,"ERROR!\nTry re-pair");
      rx.new_ready=false;
      return;
    }
    Serial.println("Received valid pair message");
    // Genuine pairing message so...
    memcpy(main_state.partner,rx.mac_addr,6);
    
    // Send the echo message back directly
    strcpy(message.text,follower_echo_pair_text);



    // Add the new party as a peer
    //memcpy(peerInfo.peer_addr,&main_state.partner,6);
    //esp_now_peer_info_t leader_peer_info;
    peerInfo.channel=WIFI_CHANNEL;
    peerInfo.encrypt=false;
    memcpy(peerInfo.peer_addr,rx.mac_addr,6); // was mac not broadcast_addr
    esp_err_t add_result=esp_now_add_peer((const esp_now_peer_info_t *)&peerInfo);

    Serial.printf("Result of adding peer info: %s\n",esp_err_to_name(add_result));

    esp_err_t result=esp_now_send(main_state.partner, \
                                  (uint8_t *)&message, \
                                  sizeof(message));
    if (result != ESP_OK)
    {
        Serial.println("Error sending the echo data");
        Serial.println(esp_err_to_name(result));
    } else {

        Serial.println("Echo Sent with success");
        //
        main_state.time_offset=millis();
        main_state.is_synced=true;
        change_pairing_state(PAIRED_SYNCED,"Successful follower pairing");
        show_message(3,"Paired\nOK"); // Delay here also allows ESP-NOW send to complete before wifi switches off
        delay_with_yield(2000);
        switch_off_wifi();
        save_state();
        
    }
    rx.new_ready=false; // Flag it's now processed and we can rx another
}
void follower_syncing_rx(received_msg rx)
{
// Checks
    if (strcmp(rx.message.text,sync_message_text)!=0 || \
        memcmp(rx.mac_addr,main_state.partner,6)!=0)
    {
      const char * buffer="\n\n==================\n"
                          "ERROR - expected sync message\n"
                          "Ignoring!";
      Serial.println(buffer);
      show_message(5,"ERROR!\nTry re-sync");
      rx.new_ready=false;
      return;
    }
    // Genuine sync message so...
    
    // Send the echo message back directly
    strcpy(message.text,follower_echo_sync_text);



    // Add the new party as a peer
    //memcpy(peerInfo.peer_addr,&main_state.partner,6);
    esp_now_peer_info_t leader_peer_info;
    leader_peer_info.channel=WIFI_CHANNEL;
    leader_peer_info.encrypt=false;
    memcpy(leader_peer_info.peer_addr,main_state.partner,6);
    esp_now_add_peer((const esp_now_peer_info_t *)&leader_peer_info);

    esp_err_t result=esp_now_send(main_state.partner, \
                                  (uint8_t *)&message, \
                                  sizeof(message));
    if (result != ESP_OK)
    {
        Serial.println("Error sending the sync echo data");
        Serial.println(esp_err_to_name(result));
    } else {

        Serial.println("Echo sync Sent with success");
        //
        main_state.time_offset=millis();
        main_state.is_synced=true;
        change_pairing_state(PAIRED_SYNCED,"Successful follower sync");
        switch_off_wifi();
    }
    rx.new_ready=false; // Flag it's now processed and we can rx another
}

void OnRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  // Just places received message into last_received global to be
  // picked up by the state engine
  
  // Check we haven't got an unprocessed message waiting
  if (last_received.new_ready)
  {
    Serial.println("New incoming message blocked by another waiting to be procesed");
    return;
  }


  memcpy(&last_received.message, incomingData, sizeof(message));
  memcpy(&last_received.mac_addr,mac,6);
  last_received.new_ready=true;
  
  Serial.print("Leader Bytes received: ");
  Serial.println(len);
  Serial.print("content ");
  Serial.println(last_received.message.text);
  char buff[40];
  buff_print_mac(buff,(uint8_t*)mac);
  Serial.printf("Mac address: %s\n",buff);
 
  
}




esp_now_peer_info_t leader_peer_info;
// callback function that will be executed when data is received










void shutdown()
{

  //mark synced as false
  main_state.is_synced=false;
  //if we are paired change pairing state
  if (main_state.pairing_state==PAIRING)
  {
    change_pairing_state(BLANK_WAITING_TO_START_PAIRING,"Shutdown during pairing, return to blank");
  } 
  if (main_state.pairing_state==SYNCING || main_state.pairing_state==PAIRED_SYNCED)
  {
        change_pairing_state(PAIRED_NOT_SYNCED,"Shutdown during synching or running. revert to paired not synced");
  }
  save_state();

  switch_off_wifi();

  show_message(3,"Shutting\nDown");

  // Wait for shutdown key to be released
  while (true)
  {
    delay_with_yield(200); // Anti bounce
    if (digitalRead(PIN_FRONT_BUTTON)!=PRESSED) break;
  }
  delay_with_yield(200); //Anti bounce
  esp_sleep_enable_ext0_wakeup(WAKE_UP_PIN_DEFN,PRESSED);
  esp_deep_sleep_start();
}


void tdisplay_display_init()
{
  Serial.println("Initialising t-display screen");
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_RED);
  tft.setCursor(0,0);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.print(main_state.is_leader?"Leader":"Follower");
  delay_with_yield(2000);
}
#ifdef BOARD_TYPE_M5STICKC
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

    void M5_draw_leader(bool is_leader)
    {
      int32_t width=is_leader?90:112;
      int32_t top_left_x=80-(width/2);
      int32_t top_left_y=0;
      
      M5.Lcd.fillRect(top_left_x,top_left_y,width,22,BLACK);
      M5.Lcd.drawRect(top_left_x,top_left_y,width,22,is_leader?GREEN:RED);
      M5.Lcd.setCursor(top_left_x+2,top_left_y+3);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setTextColor(is_leader?RED:GREEN);
      M5.Lcd.print(is_leader?"Leading":"Following");

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

#endif






void update_alerts(uint16_t phase_ms)
{
  
  uint32_t offset_now=((millis()-main_state.time_offset)/1000); // Alternate every second
  buzzing=main_state.buzz_enabled & main_state.is_synced & ((offset_now & 0x0001)^main_state.is_leader); // Only buzz when synced
  
  #ifdef ENABLE_BUZZING
  //digitalWrite(PIN_VIBRATION,buzzing?VIBRATING:VIBE_STOPPED);
  ledcWrite(PWM_CHANNEL,PWM_LEVEL);
  #endif
  #ifdef ENABLE_LED
  
  digitalWrite(PIN_LED,(!buzzing) | (!main_state.led_enabled)); // Low is LED on 
 
  #endif
}

button_state check_button(uint8_t button_pin)
{
  button_state response;
 if (digitalRead(button_pin)==PRESSED)
  {
    delay_with_yield(200); // Anti-bounce
    uint16_t count=0;
    while (digitalRead(button_pin)==PRESSED && count<(VERY_LONG_BUTTON_THRESHOLD+1000))
    {
      count+=10;
      delay_with_yield(10);
    }
    Serial.printf("Button pressed for : %d ms\n\n",count);
    delay_with_yield(500);
    response.pressed=true;
    response.press_length_ms=count;
  } else {
    response.pressed=false;
    response.press_length_ms=0;
  }
  return response;
}


void esp_now_startup(bool broadcast=false)
{
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  radio_on=true;
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  if (main_state.is_leader)
  {
    esp_now_register_send_cb(OnLeaderSent);
    esp_now_register_recv_cb(OnRecv);
  } else {
    esp_now_register_recv_cb(OnRecv);
    esp_now_register_send_cb(OnFollowerSent);
  }



  if (broadcast)
  {
    Serial.println("Starting broadcast channel");
    
    peerInfo.channel=WIFI_CHANNEL;
    memcpy(peerInfo.peer_addr, broadcast_addr, 6); // was peer_address
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
      Serial.println("Failed to add peer");
      return;
    }
  } else {
    Serial.println("Starting peer-to-peer channel");
    
    memcpy(peerInfo.peer_addr, main_state.partner, 6);
    peerInfo.channel = WIFI_CHANNEL;  
    peerInfo.encrypt = false;

   
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
      Serial.println("Failed to add peer");
      return;
    }

  }
  

      

}

void pairing_init()
{
  esp_now_startup(true);
}
void leader_pairing_init()
{
  //esp_now_register_send_cb(OnDataSent);
  pairing_init();
}
void follower_pairing_init()
{
  //esp_now_register_send_cb(OnDataSent);
  pairing_init();
}

void syncing_init()
{
  esp_now_startup(false);
}

void leader_syncing_init()
{
  //esp_now_register_send_cb(OnDataSent);
  syncing_init();
}
  
  

void follower_syncing_init()
{
  //esp_now_register_recv_cb(OnDataRecv); 
  syncing_init();
  
}




void leader_send_pair_request()
{
    Serial.println("Attempting to call to follower...");
    if (!radio_on)
    {
      Serial.println("Switching on radio...");
      leader_pairing_init();

    } else {
      Serial.println("Radio is on");
    }
    leader_peer_info.channel=WIFI_CHANNEL;
    leader_peer_info.encrypt=false;
    memcpy(leader_peer_info.peer_addr,broadcast_addr,6);
    esp_now_add_peer((const esp_now_peer_info_t *)&leader_peer_info);

    strcpy(message.text,pair_message_text);
    // Was sent to pair_address, now it's broadcast
    esp_err_t result=esp_now_send(broadcast_addr,
                              (uint8_t *) &message,
                              sizeof(message));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.printf("Error sending the data: %s\n",esp_err_to_name(result));
    }
}

void start_pairing()
{
    if (main_state.pairing_state!=PAIRING)
    {
      change_pairing_state(PAIRING,"Start pairing called");
    }
    main_state.is_synced=false;
    if (main_state.is_leader)
    {
      leader_pairing_init();
      leader_send_pair_request();
    } else {
      follower_pairing_init();
    }
    

}

void leader_send_sync_request()
{
    Serial.println("Attempting to call to follower...");
    strcpy(message.text,sync_message_text);
    // Was sent to pair_address, now it's broadcast
    
    esp_err_t result=esp_now_send(main_state.partner,
                              (uint8_t *) &message,
                              sizeof(message));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }
}



uint32_t pair_loop_tries=0;

void start_syncing()
{
    if (main_state.pairing_state!=SYNCING)
    {
      change_pairing_state(SYNCING,"Synching started without changing state before");
    }
    main_state.is_synced=false;
    if (main_state.is_leader)
    {
      main_state.is_synced=false;// this will change when we get synced
      leader_syncing_init();
      leader_send_sync_request();
    } else {
      main_state.is_synced=false;
      follower_syncing_init();
    }
}



void update_state()
{
  // This function defines the behaviour of the device. It is called many times each second
  // to consider how things need to change.


  //uint32_t state_duration=(main_state.state_change_time-millis());
  // Changes state based on button presses and progress pairing

  /* 
  
      Button behaviour
      ================


        Terminology:
          Pairing- the process of learning and remembering the partner device. IN theory should only need to happen once
          Syncing- the process of two previously paired devices getting their buzzing in sync. This has to happen every time the devices switch on.


        Objectives:
          To do what the user intuitively expects without them having to really understand this stuff!

        Problems:
          Feedback: How will the user know what is happening if we don't have a screen?
      
        When switched off (deep-sleep mode for now)

        When in deep-sleep, any press on a button should wake the device
        (not coded here, that'll have to come later).
            If paired it will automatically try to sync for 1 minute before going back to sleep.
            If not paired it will go into pairing mode for 1 minute before going back to sleep.

        When awake:

            When in pairing mode:
              A short press causes it to stop pairing and go back to deep-sleep
              A very long press causes it to factory reset

            When in syncing mode:
              A short press causes it to stop syncing and go back to deep-sleep
              A long press causes it to re-enter pairing mode
              A very long press causes it to factory reset

            When in paired and synced mode:
              A short press causes it to switch off/deep-sleep
              A long press causes it to switch off
              A very long press causes it to factory reset




  */

  // Convert button presses to flags:
  bool short_press=false;
  
  bool long_press=false;
  bool very_long_press=false;

  if (front_button.pressed)
  {
    if (front_button.press_length_ms>VERY_LONG_BUTTON_THRESHOLD)
    {
      very_long_press=true;
    } else {
      if (front_button.press_length_ms>SHORT_BUTTON_THRESHOLD)
      {
        long_press=true;
      } else {
        short_press=true;
      }
    }
  }

  if (very_long_press)
  {// Factory reset option
    change_pairing_state(BLANK_WAITING_TO_START_PAIRING,"Factory reset selected");
    show_message(3,"Factory\nReset");
    main_state.is_synced=false;
    memset(&main_state.partner,0,6);
    save_state();
    show_message(4,"Factory\nReset");
    Serial.println("Pairing deleted, shutting down....");
    delay_with_yield(2000);
    shutdown();
  }

  if (short_press)
  {// Just switch off
    save_state();
    Serial.println("Switching off now...");
    delay_with_yield(1000);
    shutdown();

  }

  if (main_state.pairing_state==PAIRED_SYNCED && long_press)
  {
    // A long press here should be a switch off case
      save_state();
      shutdown();

  }

  if (main_state.pairing_state==SYNCING && long_press)
  { // Long press during syncing means enter pairing mode
    change_pairing_state(PAIRING,"long press during sync");
    main_state.is_synced=false;
    memcpy(main_state.partner,blank_partner,6);
    start_pairing();
    return;
  }

  if (main_state.is_leader)
  {
    switch (main_state.pairing_state)
    {
      case BLANK_WAITING_TO_START_PAIRING:
        change_pairing_state(PAIRING,"Auto-blank-to-pairing");
        main_state.is_synced=false;
        pair_loop_tries=0;
        start_pairing();
        break;

      case PAIRING:
        // Check for inbound message
        pair_loop_tries++;
        if (last_received.new_ready)
        {
          leader_pairing_rx(last_received);
          last_received.new_ready=false;
        } else {
          if (pair_loop_tries>600)
          {
            // Give up
            change_pairing_state(BLANK_WAITING_TO_START_PAIRING,"Timed out to pair");
            Serial.println("Pairing failed, reverting to blank state");
            save_state();
            switch_off_wifi();
            shutdown();
            break;
          }
          if ((pair_loop_tries % 20)==0)
          {
            leader_send_pair_request();
          }

        }
        break;

      case PAIRED_NOT_SYNCED:
          // State expected after pairing on new reboot
          // Automatically start the syncing process
          start_syncing();
          pair_loop_tries=0;// counting for resends
          change_pairing_state(SYNCING,"Leader starting to sync");
          break;

      case SYNCING:
        pair_loop_tries++;
        if (last_received.new_ready)
        {
          leader_syncing_rx(last_received);
          last_received.new_ready=false;
        } else {
          if (pair_loop_tries>600)
          {
            // Give up
            change_pairing_state(PAIRED_NOT_SYNCED,"Timed out to sync");
            Serial.println("Sync failed, packing up");
            save_state();
            switch_off_wifi();
            shutdown();
            break;
          }
          if ((pair_loop_tries%20)==0)
          {
            leader_send_sync_request();// Send another request
          }
        }

        break;

      case PAIRED_SYNCED:
        // Nothing to do for the moment!

        break;
        
      case DUMMY:
        Serial.println("Wierdly, state is in dummy state!");
        break;
    }// End of leader switch
  } else {
    // Start of handling follower states
    switch (main_state.pairing_state)
    {
      case BLANK_WAITING_TO_START_PAIRING:
        change_pairing_state(PAIRING,"Auto start pairing");
        main_state.is_synced=false;
        follower_pairing_init();
        pair_loop_tries=0;
        break;

      case PAIRING:
        pair_loop_tries++;

        if (last_received.new_ready)
        {
          follower_pairing_rx(last_received);
          last_received.new_ready=false;
        }
        if (!radio_on)
        {
          esp_now_startup();
        }
      
        if (pair_loop_tries>600)
        {
          change_pairing_state(BLANK_WAITING_TO_START_PAIRING,"Timed out pairing");
          Serial.println("follower failed to pair");
          switch_off_wifi();
          save_state();
          shutdown();
        }
        break;

      case PAIRED_NOT_SYNCED:
          start_syncing();
          pair_loop_tries=0;// counting for resends
          change_pairing_state(SYNCING,"Follower starting to sync");
        break;

      case SYNCING:
        pair_loop_tries++;
        if (last_received.new_ready)
        {
          Serial.println("Possible sync message received");
          follower_syncing_rx(last_received);
          last_received.new_ready=false;
        }

        // Time out would go here
        if (pair_loop_tries>1200) // Approx 2 mins
        {
          change_pairing_state(PAIRED_NOT_SYNCED,"Timed out syncing");
          save_state();
          shutdown();
        }
        break;

      case PAIRED_SYNCED:
        // Nothing to do for the moment
        break;

      case DUMMY:
        Serial.println("Wierdly, state is in dummy state!");
        break;
    }// Emd pf follower switch
  }
}













void update_buttons()
{
 front_button=check_button(PIN_FRONT_BUTTON);
 #ifdef PIN_SIDE_BUTTON
 side_button=check_button(PIN_SIDE_BUTTON);
 #endif
}


void setup() {
  // put your setup code here, to run once:

  
  Serial.begin(115200);
  setCpuFrequencyMhz(80);// Slow down the cores to save a little juice
  
  delay_with_yield(300);
  Serial.println("booted");



  pinMode(PIN_VIBRATION,OUTPUT);
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_VIBRATION,VIBE_STOPPED);
  pinMode(PIN_FRONT_BUTTON,INPUT);
  #ifdef PIN_SIDE_BUTTON
  pinMode(PIN_SIDE_BUTTON,INPUT);
  #endif
  delay_with_yield(500);
  
  // Load the state from preferences
  
  preferences.begin("altanx"); // Load the preferences

  // Set up pwm
  ledcSetup(PWM_CHANNEL,PWM_FREQ,PWM_RESOLUTION);
  ledcAttachPin(PIN_VIBRATION,PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL,0);

  if (preferences.isKey("syststate") && saving_peer_info) // Disabled during development
  {
      Serial.println("Loading saved state");
      // There is a saved state, so load it
      uint8_t temp_buffer[30]; // Actual state is only about 15 bytes
      preferences.getBytes("syststate",temp_buffer,30);
      // Now copy those bytes to the current state
      memcpy(&main_state,&temp_buffer,sizeof(main_state));
      memcpy(&old_state,&temp_buffer,sizeof(old_state));
      Serial.println("Succesfully loaded state from flash...");
      Serial.printf("\t\tIs leader: %d\n",main_state.is_leader);
      Serial.printf("\t\tIs synced: %d\n",main_state.is_synced);
      Serial.printf("\t\tPair state: %s\n",state_names[main_state.pairing_state]);

      /* Should be one of two states only:
        BLANK_WAITING_TO_START_PAIRING
        or
        PAIRED_NOT_SYNCED

        At switch on we need to start either pairing or syncing
      */

      if (main_state.pairing_state==PAIRED_NOT_SYNCED)
      {
        start_syncing();
      } 
      if (main_state.pairing_state==BLANK_WAITING_TO_START_PAIRING)
      {
        start_pairing();
      }




  } else {
    // Write in a new blank state... and auto start pairing

    
    main_state.is_leader=is_leader_def;
    main_state.is_synced=false;
    main_state.pairing_state=PAIRING;
    main_state.led_enabled=false;
    main_state.buzz_enabled=true;
    memcpy(main_state.partner,blank_partner,6);
    main_state.time_offset=0;
    save_state();



    

    memcpy(&old_state,&main_state,sizeof(main_state));
    Serial.println("Successfully put dummy state into the store");

  }
  
  
  
  
  
  #ifdef ENABLE_DISPLAY

    #ifdef BOARD_TYPE_M5STICKC
    M5_display_init();
    //NB This will need fixing to work with the M5StickC again, not expecting to do that at the moment
    //update_display(main_state,true);// Force update even if nothing is changed
    #endif
    #ifdef BOARD_TYPE_TDISPLAY
    tdisplay_display_init();
    //show_message(3,"Altanx\n======\nHello");
    //update_display(main_state,true);// Force update even if nothing is changed
    Serial.println("Returned from displaying welcome message");
    #endif
  #endif
  
  // Serial.print("This device MAC is : ");
  // delay(200);
  // Serial.println(WiFi.macAddress());
  // delay(200);

  Serial.printf("Device %s leader?\n",main_state.is_leader?"IS":"ISN'T");



}// end of setup


void loop()
{
  // put your main code here, to run repeatedly:
  phase_ms=(millis()-main_state.time_offset) % BUZZ_PERIOD_MS; // Will go from 0 to buzz period every cycle
  
  update_alerts(phase_ms); // does buzzing and or LED
  update_buttons(); // reads button states
  update_state(); // looks for state changes
  #ifdef ENABLE_DISPLAY
  update_display(main_state);
  #endif
  old_state=main_state;
  //Serial.printf("offst:  %d,",offset_now);
  Serial.printf("buzz:%d,   ",buzzing);
  Serial.printf("fr_but: %d ,",front_button.pressed);
  Serial.printf("buzz_en:%d ,",main_state.buzz_enabled);
  Serial.printf("mstr: %d,  ,",main_state.is_leader);
  Serial.printf("state: %s,  ",state_names[main_state.pairing_state]);
  Serial.printf("sync: %d    ",main_state.is_synced);
  Serial.printf("Radio: %d   ",radio_on);
  Serial.println();
  delay_with_yield(LOOP_DELAY_MS);

}
  
  
  
  
  

