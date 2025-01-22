#include <FastLED.h>
#include <EEPROM.h>
#include <driver/adc.h>
#include "esp32-hal-adc.h" // Bibliotheque pour l'ADC
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// EEPROM.write(address, value); // 512 adresses 255 bits 

// t'imagine t'ajoute la partie lumizic ?... :))))

//pr les coms leds : A -> B

//--------------------------------------------------------------------------------------------------------------------------//
//Init Variable
//--------------------------------------------------------------------------------------------------------------------------//

#define       LED_PIN             13                                  // Pin de commande LED
#define       NUM_LEDS            13                                  // Nombre de Leds 
#define       LED_TYPE            WS2812
#define       COLOR_ORDER         GRB
#define       EEPROM_SIZE         4                                   // car j'utilise pas le 0 bah mtn si
//#define       TPS_INT_TIMER_ADC   100000                              // valeur en ms pour le timer 1, ici 100ms
CRGB          leds[NUM_LEDS];
const char    Button           =   32;                                // Boutton d'action
const char    Relais_Led_BTN   =   33;                                // relais out    
const char    Relais_ST_HAUT   =   27;                                // Relais btn haut volant
const char    Relais_ST_BAS    =   14;                                // Relais btn bas volant
const char    ADC_PORT         =   15;                                // Pin pour le ADC
      char    Cpt_Button       =   00 ;                               // comptage action boutton   
      char    Rouge;      
      char    Vert;     
      char    Bleu;   
      char    mode_color;
      char    NBS_MODE = 5;
      char    Limite; 
      float   Voltage_Steering_Wheels = 0;
      int     VoltageRead;


//--------------------------------------------------------------------------------------------------------------------------//
// CORE 1 -  MODE Commande au volant
//--------------------------------------------------------------------------------------------------------------------------//

void ReadVoltage(void * pvParameters) 
{
  while(1) // boucle infinie
  { 
    // Lire et convertir la valeur de la broche
    VoltageRead = analogRead(ADC_PORT); 
    Voltage_Steering_Wheels = (VoltageRead / 4095.0) * 3.3;

    // Afficher la tension
    Serial.print("La tension est : ");
    Serial.println(Voltage_Steering_Wheels);
    
    if(Voltage_Steering_Wheels > 1.25 && Voltage_Steering_Wheels < 1.55)  //btn haut 280ohm
    {
      Serial.print("ST HAUT PRESSED");
      digitalWrite(Relais_ST_HAUT, HIGH);
      delay(450);
      digitalWrite(Relais_ST_HAUT, LOW);
    }

    if(Voltage_Steering_Wheels > 2.05 && Voltage_Steering_Wheels < 2.35)  //btn bas 450 ohm
    {
      Serial.print("ST BAS PRESSED");
      digitalWrite(Relais_ST_BAS, HIGH);
      delay(400);
      digitalWrite(Relais_ST_BAS, LOW);
    }
    
    vTaskDelay(pdMS_TO_TICKS(35)); // delay de 15ms
  }
}

void Principal(void * pvParameters) 
{
 while(1) 
 {
  if (digitalRead(Button) == HIGH)
  //----------------------------------------------------------------------------------------------------BTN PRESED ?-------------------//
    {
      { //******************************************* 1 Pression
          Serial.println("BOUTON...\n"); 
      
          if (digitalRead(Button) == HIGH)
          {
            delay(200);
            FastLED.clear(true);
            digitalWrite(Relais_Led_BTN, LOW);
            delay(200);
            digitalWrite(Relais_Led_BTN, HIGH);
            SetAllLed(255,255,255);
            delay(200);
            digitalWrite(Relais_Led_BTN, LOW);
            FastLED.clear(true);
            delay(200);
            digitalWrite(Relais_Led_BTN, HIGH);
            SetAllLed(255,255,255);
            delay(500);
          }
      
          if (digitalRead(Button) == LOW)
          {
          mode_color++;
          if(mode_color>NBS_MODE) { mode_color = 1;}
          EEPROM.write(0, mode_color); EEPROM.commit();
          }
          FastLED.clear(true);
      }
      //******************************************* 1 Pression END
          //******************************************* LONG Pression
      
                                   
        if (digitalRead(Button) == HIGH)
        {    
            Cpt_Button ++; // pour connaitre quelle couleur regler
            if(Cpt_Button>3) { Cpt_Button = 1;}
            delay(200);
            digitalWrite(Relais_Led_BTN, LOW);
            delay(600);
            digitalWrite(Relais_Led_BTN, HIGH);
            if (Cpt_Button == 1) { Rouge=0; SetAllLed(255,0,0) ; delay (300) ; SetAllLed(255,0,0) ; delay (300) ; SetAllLed(255,0,0) ; delay (300) ;    EEPROM.write(1, 0); EEPROM.commit();}
            if (Cpt_Button == 2) { Vert=0;  SetAllLed(0,255,0) ; delay (300) ; SetAllLed(0,255,0) ; delay (300) ; SetAllLed(0,255,0) ; delay (300) ;    EEPROM.write(2, 0); EEPROM.commit();}
            if (Cpt_Button == 3) { Bleu=0;  SetAllLed(0,0,255) ; delay (300) ; SetAllLed(0,0,255) ; delay (300) ; SetAllLed(0,0,255) ; delay (300) ;    EEPROM.write(3, 0); EEPROM.commit();}
            delay(600);
            digitalWrite(Relais_Led_BTN, LOW);
            FastLED.clear(true); 
            delay(600);
            digitalWrite(Relais_Led_BTN, HIGH);
            if (Cpt_Button == 1) { Rouge=0; SetAllLed(255,0,0) ; delay (300) ; SetAllLed(255,0,0) ; delay (300) ; SetAllLed(255,0,0) ; delay (300) ;    EEPROM.write(1, 0); EEPROM.commit();}
            if (Cpt_Button == 2) { Vert=0;  SetAllLed(0,255,0) ; delay (300) ; SetAllLed(0,255,0) ; delay (300) ; SetAllLed(0,255,0) ; delay (300) ;    EEPROM.write(2, 0); EEPROM.commit();}
            if (Cpt_Button == 3) { Bleu=0;  SetAllLed(0,0,255) ; delay (300) ; SetAllLed(0,0,255) ; delay (300) ; SetAllLed(0,0,255) ; delay (300) ;    EEPROM.write(3, 0); EEPROM.commit();}
            delay(300);
            digitalWrite(Relais_Led_BTN, LOW);
            FastLED.clear(true); 
            SetAllLed(0,0,0);
            delay(300);
            digitalWrite(Relais_Led_BTN, HIGH);
            delay(300);
            digitalWrite(Relais_Led_BTN, LOW);
            delay(300);
            digitalWrite(Relais_Led_BTN, HIGH);
            if (Cpt_Button == 1) { Rouge=0; SetAllLed(255,0,0) ; delay (300) ; SetAllLed(255,0,0) ; delay (300) ; SetAllLed(255,0,0) ; delay (300) ;    EEPROM.write(1, 0); EEPROM.commit();}
            if (Cpt_Button == 2) { Vert=0;  SetAllLed(0,255,0) ; delay (300) ; SetAllLed(0,255,0) ; delay (300) ; SetAllLed(0,255,0) ; delay (300) ;    EEPROM.write(2, 0); EEPROM.commit();}
            if (Cpt_Button == 3) { Bleu=0;  SetAllLed(0,0,255) ; delay (300) ; SetAllLed(0,0,255) ; delay (300) ; SetAllLed(0,0,255) ; delay (300) ;    EEPROM.write(3, 0); EEPROM.commit();}
      }
      
      while  (digitalRead(Button) == HIGH)
      {                                                      // 8 choix de luminosité possible 8^3   soit 512 possibilité
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
      //----------------------------------------------------------------------------------------------------END BTN PRESED ?-------------------//
    //----------------------------------------------------------------------------------------------------MODE LED-------------------//
    
    if(mode_color==1)                                                                           //MODE 1 - STATIC
    {
      SetAllLed(Rouge,Vert,Bleu);
    }

    if(mode_color==2)                                                           //MODE 2 - ALLER -> Puis OFF ->
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
          leds[i] = CRGB ( 0, 0, 0);                          
          FastLED.show();                                       // Utilisation de l'algo FastLed
          delayMicroseconds(5);                                 // delay de 5  us
          delay(60);                                            // delay de 60 ms
      }
    }
    
    if(mode_color==3)                                                             //MODE 3 - jsp ?
    {
      for (int i = 0; i <= NUM_LEDS-1; i++) 
      {          
          FastLED.clear(true);                                                // better than set all led
          leds[i] = CRGB ( (Rouge), (Vert), (Bleu));                          
          leds[i+1] = CRGB ( (Rouge), (Vert), (Bleu));                          
          leds[i+2] = CRGB ( (Rouge), (Vert), (Bleu));                          
          FastLED.show();                                       // Utilisation de l'algo FastLed
          delayMicroseconds(5);                                 // delay de 5  us
          delay(80);                                            // delay de 60 ms
      }
    }
    
    if(mode_color==4)                                     //MODE 4 - luminosité --- puis +++ "rapide"
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
    
    if(mode_color==5)                                     //MODE 5 - luminosité --- puis +++ mais "lent"
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
  //----------------------------------------------------------------------------------------------------END MODE LED-------------------//
 }

  
}

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
    // Configurer l'ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten((adc1_channel_t)ADC_PORT, ADC_ATTEN_DB_11);

  // Configurer les broches de relais comme sorties
  pinMode(Relais_ST_HAUT, OUTPUT);
  pinMode(Relais_ST_BAS, OUTPUT);
  
             

  // Confg des mémoire RGB et mode au boot
  EEPROM.begin(EEPROM_SIZE);
  Rouge           =   EEPROM.read(1);
  Vert            =   EEPROM.read(2);
  Bleu            =   EEPROM.read(3);
  mode_color      =   EEPROM.read(0);
  
  pinMode(Button, INPUT_PULLUP);                                             
  pinMode(Relais_Led_BTN, OUTPUT); 

 // FastLED.addLeds<LED_TYPE,LED_PIN,COLOR_ORDER>(leds,NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  Serial.begin(115200);
  Serial.println("BOOT...\n");   
  
  SetAllLed(Rouge,Vert,Bleu);
  
  for (int y = 0; y <= 2; y++)
  {
      for (int i = 0; i <= 255; i++)          // led brightness rise down -> 0
      {         
           FastLED.setBrightness(255-i);      
           FastLED.show(); 
           delay(1);
      }
      for (int i = 0; i <= 255; i++)          // led brightness rise up -> 255
      {         
           FastLED.setBrightness(i);          
           FastLED.show(); 
           delay(1);
      }
  }
        delay(20);
        FastLED.clear(true);
  
        digitalWrite(Relais_Led_BTN, HIGH);
        digitalWrite(Relais_ST_BAS, LOW);
        digitalWrite(Relais_ST_HAUT, LOW);

  // Créer une tâche qui s'exécute sur le noyau 1.
  xTaskCreatePinnedToCore
  (
    ReadVoltage,   /* La fonction à exécuter */
    "ReadVoltage", /* Nom de la tâche */
    10000,         /* Taille de la pile pour la tâche */
    NULL,          /* Paramètre passé à la tâche */
    1,             /* Priorité de la tâche */
    NULL,          /* Handle de la tâche */
    0              /* Le cœur sur lequel la tâche doit être exécutée */
  );

  xTaskCreatePinnedToCore
  (
    Principal,   /* La fonction à exécuter */
    "Principal", /* Nom de la tâche */
    10000,         /* Taille de la pile pour la tâche */
    NULL,          /* Paramètre passé à la tâche */
    2,             /* Priorité de la tâche */
    NULL,          /* Handle de la tâche */
    1              /* Le cœur sur lequel la tâche doit être exécutée */
  );
}
//--------------------------------------------------------------------------------------------------------------------------//
//LOOP
//--------------------------------------------------------------------------------------------------------------------------//
void loop (void)
{}