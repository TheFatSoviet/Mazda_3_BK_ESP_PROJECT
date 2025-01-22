#include <FastLED.h>
#include <EEPROM.h>
// EEPROM.write(address, value); // 512 adresses 255 bits 


// t'imagine t'ajoute la partie lumizic ?... :))))

//--------------------------------------------------------------------------------------------------------------------------//
//Init Variable
//--------------------------------------------------------------------------------------------------------------------------//

#define       LED_PIN             13                                  // Pin de commande LED
#define       NUM_LEDS            13                                  // Nombre de Leds 
#define       LED_TYPE            WS2812
#define       COLOR_ORDER         GRB
#define EEPROM_SIZE 4                                                   // car j'utilise pas le 0 bah mtn si
CRGB          leds[NUM_LEDS];
const char    Button          =   32;                                  // Boutton d'action
const char    Relais_Led_BT   =   33;                                  // Boutton d'action     
      char    Cpt_Button      =   0 ;                                  // comptage action boutton   
      char    Rouge;      
      char    Vert;     
      char    Bleu;   
      char    mode_color;
      char    NBS_MODE = 5;
      char    Limite; 

//--------------------------------------------------------------------------------------------------------------------------//
//SET ALL LED
//--------------------------------------------------------------------------------------------------------------------------//
void SetAllLed(char R,char G,char B)
{ 
    for (int i = 0; i <= (NUM_LEDS-1); i++)              
    {
          leds[i] = CRGB ( R, G, B);
          FastLED.show();
          delay(2);
    }
}


//---------------- ----------------------------------------------------------------------------------------------------------//
//SETUP
//--------------------------------------------------------------------------------------------------------------------------//
void setup() 
{
  EEPROM.begin(EEPROM_SIZE);
            Rouge           =   50;
            Vert            =   EEPROM.read(2);
            Bleu            =   180;
            
            mode_color     =   EEPROM.read(0);
  
    pinMode(Button, INPUT_PULLUP);                                             
    pinMode(Relais_Led_BT, OUTPUT); 
  
    FastLED.addLeds<LED_TYPE,LED_PIN,COLOR_ORDER>(leds,NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    Serial.begin(115200);
    Serial.println("BOOT...\n");   
  
  
    
    SetAllLed(Rouge,Vert,Bleu);
  
  for (int y = 0; y <= 2; y++)
  {
      for (int i = 0; i <= 255; i++) 
                      {         
                           FastLED.setBrightness(255-i);                  // delay de 60 ms
                           FastLED.show(); 
                           delay(1);
                      }
      for (int i = 0; i <= 255; i++) 
                      {         
                           FastLED.setBrightness(i);                  // delay de 60 ms
                           FastLED.show(); 
                           delay(1);
                      }
  }
  
        delay(20);
        FastLED.clear(true);
  
        digitalWrite(Relais_Led_BT, HIGH);
    
}

//--------------------------------------------------------------------------------------------------------------------------//
//LOOP
//--------------------------------------------------------------------------------------------------------------------------//
  void loop (void)
{  

  //SetAllLed(////
 if (digitalRead(Button) == HIGH)
  {
     {
        Serial.println("BOUTON...\n"); 
    
        if (digitalRead(Button) == HIGH)
         {
          delay(200);
           FastLED.clear(true);
            digitalWrite(Relais_Led_BT, LOW);
            delay(200);
            digitalWrite(Relais_Led_BT, HIGH);
            SetAllLed(100,0,0);
            delay(200);
            digitalWrite(Relais_Led_BT, LOW);
            FastLED.clear(true);
            delay(200);
            digitalWrite(Relais_Led_BT, HIGH);
            SetAllLed(100,0,0);
            delay(500);
        }
    
        if (digitalRead(Button) == LOW)
        {
           mode_color++;
           if(mode_color>NBS_MODE) { mode_color = 1;}
           EEPROM.write(0, mode_color); EEPROM.commit();}
           FastLED.clear(true);
     }
    
    
      Cpt_Button ++;
      if(Cpt_Button>3) { Cpt_Button = 1;}
      delay(200);                                      
      if (digitalRead(Button) == HIGH)
      {    
          Cpt_Button ++;
          if(Cpt_Button>3) { Cpt_Button = 1;}
          delay(100);
          digitalWrite(Relais_Led_BT, LOW);
          delay(600);
          digitalWrite(Relais_Led_BT, HIGH);
          if (Cpt_Button == 1) { Rouge=0; SetAllLed(255,0,0) ; delay (300) ; SetAllLed(255,0,0) ; delay (300) ; SetAllLed(255,0,0) ; delay (300) ;    EEPROM.write(1, 0); EEPROM.commit();}
          if (Cpt_Button == 2) { Vert=0;  SetAllLed(0,255,0) ; delay (300) ; SetAllLed(0,255,0) ; delay (300) ; SetAllLed(0,255,0) ; delay (300) ;    EEPROM.write(2, 0); EEPROM.commit();}
          if (Cpt_Button == 3) { Bleu=0;  SetAllLed(0,0,255) ; delay (300) ; SetAllLed(0,0,255) ; delay (300) ; SetAllLed(0,0,255) ; delay (300) ;    EEPROM.write(3, 0); EEPROM.commit();}
          delay(600);
          digitalWrite(Relais_Led_BT, LOW);
          FastLED.clear(true); 
          delay(600);
          digitalWrite(Relais_Led_BT, HIGH);
          if (Cpt_Button == 1) { Rouge=0; SetAllLed(255,0,0) ; delay (300) ; SetAllLed(255,0,0) ; delay (300) ; SetAllLed(255,0,0) ; delay (300) ;    EEPROM.write(1, 0); EEPROM.commit();}
          if (Cpt_Button == 2) { Vert=0;  SetAllLed(0,255,0) ; delay (300) ; SetAllLed(0,255,0) ; delay (300) ; SetAllLed(0,255,0) ; delay (300) ;    EEPROM.write(2, 0); EEPROM.commit();}
          if (Cpt_Button == 3) { Bleu=0;  SetAllLed(0,0,255) ; delay (300) ; SetAllLed(0,0,255) ; delay (300) ; SetAllLed(0,0,255) ; delay (300) ;    EEPROM.write(3, 0); EEPROM.commit();}
          delay(300);
          digitalWrite(Relais_Led_BT, LOW);
          FastLED.clear(true); 
          SetAllLed(0,0,0);
          delay(300);
          digitalWrite(Relais_Led_BT, HIGH);
          delay(300);
          digitalWrite(Relais_Led_BT, LOW);
          delay(300);
          digitalWrite(Relais_Led_BT, HIGH);
          if (Cpt_Button == 1) { Rouge=0; SetAllLed(255,0,0) ; delay (300) ; SetAllLed(255,0,0) ; delay (300) ; SetAllLed(255,0,0) ; delay (300) ;    EEPROM.write(1, 0); EEPROM.commit();}
          if (Cpt_Button == 2) { Vert=0;  SetAllLed(0,255,0) ; delay (300) ; SetAllLed(0,255,0) ; delay (300) ; SetAllLed(0,255,0) ; delay (300) ;    EEPROM.write(2, 0); EEPROM.commit();}
          if (Cpt_Button == 3) { Bleu=0;  SetAllLed(0,0,255) ; delay (300) ; SetAllLed(0,0,255) ; delay (300) ; SetAllLed(0,0,255) ; delay (300) ;    EEPROM.write(3, 0); EEPROM.commit();}
     }
     
     while  (digitalRead(Button) == HIGH)
     {
                                                              // 8 choix de luminosité possible 8^3   soit 512 possibilité
        if (Cpt_Button == 1)
        {
          Rouge = Rouge+31;
          if (Rouge > 255) { Rouge = 0;}
          EEPROM.write(1, Rouge);
          EEPROM.commit();
        }
    
        if (Cpt_Button == 2)
        {
          Vert = Vert+31;
          if (Vert > 255) { Vert = 0;}
          EEPROM.write(2, Vert);
          EEPROM.commit();
        }
    
        if (Cpt_Button == 3)
        {
          Bleu = Bleu+31;
          if (Bleu > 255) { Bleu = 0;}
          EEPROM.write(3, Bleu);
          EEPROM.commit();
        }
        SetAllLed(Rouge,Vert,Bleu);
        delay(1200);
      }
  }

  
  
  if(mode_color==1)                                                                           //MODE 1
  {
    SetAllLed(Rouge,Vert,Bleu);
  }

   
  if(mode_color==2)
  {
    for (int i = 0; i <= NUM_LEDS-1; i++) 
                      {               
                          leds[i] = CRGB ( (Rouge), (Vert), (Bleu));                          
                          FastLED.show();                                       
                          delayMicroseconds(5);                                 
                          delay(60);                                            
                      }
    for (int i = 0; i <= NUM_LEDS-1; i++) 
                      {            
                          leds[i] = CRGB ( 0, 0, 0);                          // Selection Rouge 
                          FastLED.show();                                       // Utilisation de l'algo FastLed
                          delayMicroseconds(5);                                 // delay de 5  us
                          delay(60);                                            // delay de 60 ms
                      }
  }
  
  
  if(mode_color==3)
  {
  for (int i = 0; i <= NUM_LEDS-1; i++) 
                     {          
                          FastLED.clear(true);                                                // better than set all led
                          leds[i] = CRGB ( (Rouge), (Vert), (Bleu));                          // Selection Rouge 
                          leds[i+1] = CRGB ( (Rouge), (Vert), (Bleu));                          // Selection Rouge  
                          leds[i+2] = CRGB ( (Rouge), (Vert), (Bleu));                          // Selection Rouge 
                          FastLED.show();                                       // Utilisation de l'algo FastLed
                          delayMicroseconds(5);                                 // delay de 5  us
                          delay(80);                                            // delay de 60 ms
                      }
  
 /* for (int i = NUM_LEDS-1; i >= 0; i--) 
                      {         
                          FastLED.clear(true);                                                // better than set all led
                          leds[i] = CRGB ( (Rouge), (Vert), (Bleu));                          // Selection Rouge 
                          leds[i-1] = CRGB ( (Rouge), (Vert), (Bleu));                          // Selection Rouge  
                          leds[i-2] = CRGB ( (Rouge), (Vert), (Bleu));                          // Selection Rouge 
                          FastLED.show();                                       // Utilisation de l'algo FastLed
                          delayMicroseconds(5);                                 // delay de 5  us
                          delay(80);                                            // delay de 60 ms
                      }*/
  }
  
  if(mode_color==4)
  {
  SetAllLed(Rouge,Vert,Bleu);
  for (int i = 0; i <= 255; i++) 
                      {         
                           FastLED.setBrightness(255-i);                  // delay de 60 ms
                           FastLED.show(); 
                           delay(5);
                      }
  for (int i = 0; i <= 255; i++) 
                      {         
                           FastLED.setBrightness(i);                  // delay de 60 ms
                           FastLED.show(); 
                           delay(5);
                      }
   }
   
  if(mode_color==5)
  {
  SetAllLed(Rouge,Vert,Bleu);
  for (int i = 0; i <= 255; i++) 
                      {         
                           FastLED.setBrightness(255-i);                  // delay de 60 ms
                           FastLED.show(); 
                           delay(10);
                      }
  for (int i = 0; i <= 255; i++) 
                      {         
                           FastLED.setBrightness(i);                  // delay de 60 ms
                           FastLED.show(); 
                           delay(10);
                      }
                      
  }
}
