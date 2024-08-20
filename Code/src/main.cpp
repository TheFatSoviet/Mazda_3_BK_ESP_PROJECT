#include <FastLED.h>
#include <EEPROM.h>
#include <driver/adc.h>

#include "BluetoothSerial.h" // Importer la bibliothèque BluetoothSerial
// Vérifier si la bibliothèque Bluetooth est bien compilée avec le Bluetooth activé
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT; // Créer un objet BluetoothSerial

#include "esp32-hal-cpu.h"
#include "esp32-hal.h" //CPU ch et set freq

#include "esp32-hal-adc.h" // Bibliotheque pour l'ADC

#include "freertos/FreeRTOS.h" //?
#include "freertos/task.h"     //?

#include "esp_freertos_hooks.h" // pour hook le idle

// Define pour ADC
#define ADC_PIN_36 adc1_channel_t::ADC1_CHANNEL_0 // Voltage Positive
#define ADC_PIN_39 adc1_channel_t::ADC1_CHANNEL_3 // Voltage Negative
#define ADC_PIN_32 adc1_channel_t::ADC1_CHANNEL_4
#define ADC_PIN_33 adc1_channel_t::ADC1_CHANNEL_5
#define ADC_PIN_34 adc1_channel_t::ADC1_CHANNEL_6
#define ADC_PIN_35 adc1_channel_t::ADC1_CHANNEL_7

#define ADC_PIN_04 adc2_channel_t::ADC2_CHANNEL_0
#define ADC_PIN_02 adc2_channel_t::ADC2_CHANNEL_2
#define ADC_PIN_15 adc2_channel_t::ADC2_CHANNEL_3
#define ADC_PIN_13 adc2_channel_t::ADC2_CHANNEL_4
#define ADC_PIN_12 adc2_channel_t::ADC2_CHANNEL_5
#define ADC_PIN_14 adc2_channel_t::ADC2_CHANNEL_6
#define ADC_PIN_27 adc2_channel_t::ADC2_CHANNEL_7
#define ADC_PIN_25 adc2_channel_t::ADC2_CHANNEL_8
#define ADC_PIN_26 adc2_channel_t::ADC2_CHANNEL_9

//--------------------------------------------------------------------------------------------------------------------------//
// Init Variable
//--------------------------------------------------------------------------------------------------------------------------//
#define LED_PIN 2  // Pin de commande LED
#define NUM_LEDS 13 // Nombre de Leds
#define LED_TYPE WS2812
#define COLOR_ORDER GRB
#define EEPROM_SIZE 4 // Taille éspace eeprom en byte je crois
CRGB leds[NUM_LEDS];

#define PIN_RELAIS_ACTION_BOUTONS_VOLANT_HAUT 26
#define PIN_RELAIS_ACTION_BOUTONS_VOLANT_BAS 25
#define PIN_BOUTON_INPUT_GESTION_LED 32
#define PIN_RELAIS_GESTION_LED 33

#define PIN_DECLANCHEMENT_ABAISSEMENT_RETRO 22 // feu recule quoi
#define PIN_CONTROLE_RELAIS_RETRO_Relay_1 5   // cable jaune 
#define PIN_CONTROLE_RELAIS_RETRO_Relay_2 18  // cable blanc
#define PIN_CONTROLE_RELAIS_RETRO_Relay_3 19  // cable rouge
#define PIN_CONTROLE_RELAIS_RETRO_Relay_4 21  // cable noir

typedef struct
{
  float Tension_Bus_Volant;
  int Vitesse_Tension;
  int Vitesse_PWM;
  bool Connexion_BT;
  float Charge_CPU;

} LogVariable;

// Déclaration de la structure globale
LogVariable LogVariable_1;

//charge cpu
// static TickType_t idleStartTime = 0;
// static TickType_t idleEndTime = 0;


// FONCTIONS :
void SetAllLed(char R, char G, char B);
void printCpuFrequency(void);
void setCpuFreq(uint32_t freq);
bool myIdleHook(void);
void ReadVoltage(void *pvParameters);
void Retro_Motor(void *pvParameters);
void Gestion_Led(void *pvParameters);
void Gestion_Led(void *pvParameters);
void displayIdleTask(void *pvParameters);
void Gestion_LOGs(void *pvParameters);
float mesurerPourcentagePWM(int pwmPin);

// int calcule_Vitesse_PWM(int pwmPin, unsigned long pwmMin, unsigned long pwmMax, int vitesseBas, int vitesseHaut);
// int calcule_Vitesse_Voltage(int analogPin, float voltMin, float voltMax, int vitesseMin, int vitesseMax);

bool myIdleHook(void)
{
 // idleStartTime = xTaskGetTickCount(); // Incrémenter chaque fois que la tâche Idle s'exécute
  return true;
}

// Fonction pour obtenir la fréquence actuelle du CPU
void printCpuFrequency(void)
{
  uint32_t freq = getCpuFrequencyMhz();
  Serial.print("Current CPU Frequency: ");
  Serial.print(freq);
  Serial.println(" MHz");
}

void setCpuFreq(uint32_t freq)
{
  // Attempt to set CPU frequency and print result
  if (setCpuFrequencyMhz(freq))
  {
    Serial.print("Frequency set to: ");
  }
  else
  {
    Serial.print("Failed to set frequency to: ");
  }
 // Serial.print(freq);
 // Serial.println(" MHz");
}

void SetAllLed(char R, char G, char B)
{
  for (int i = 0; i <= (NUM_LEDS - 1); i++)
  {
    leds[i] = CRGB(R, G, B);
    FastLED.show();
    delay(2);
  }
}

void Gestion_Led(void *pvParameters)
{
  const char Button = PIN_BOUTON_INPUT_GESTION_LED;   // Boutton d'action
  const char Relais_Led_BTN = PIN_RELAIS_GESTION_LED; // relais out
  
  char Rouge;
  char Vert;
  char Bleu;
  char Cpt_Button = 00; // comptage action boutton
  char mode_color;
  char NBS_MODE = 5;
  // SETUP
  pinMode(Button, INPUT_PULLUP);
  pinMode(Relais_Led_BTN, OUTPUT);
  digitalWrite(Relais_Led_BTN, HIGH);

  // Confg des mémoire RGB et mode au boot
  EEPROM.begin(EEPROM_SIZE);
  mode_color = EEPROM.read(0);
  Rouge = EEPROM.read(1);
  Vert = EEPROM.read(2);
  Bleu = EEPROM.read(3);

  SetAllLed(Rouge, Vert, Bleu);

  while (1)
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
          SetAllLed(255, 255, 255);
          delay(200);
          digitalWrite(Relais_Led_BTN, LOW);
          FastLED.clear(true);
          delay(200);
          digitalWrite(Relais_Led_BTN, HIGH);
          SetAllLed(255, 255, 255);
          delay(500);
        }

        if (digitalRead(Button) == LOW)
        {
          mode_color++;
          if (mode_color > NBS_MODE)
          {
            mode_color = 1;
          }
          EEPROM.write(0, mode_color);
          EEPROM.commit();
        }
        FastLED.clear(true);
      }
      //******************************************* 1 Pression END
      //******************************************* LONG Pression

      if (digitalRead(Button) == HIGH)
      {
        Cpt_Button++; // pour connaitre quelle couleur regler
        if (Cpt_Button > 3)
        {
          Cpt_Button = 1;
        }
        delay(200);
        digitalWrite(Relais_Led_BTN, LOW);
        delay(600);
        digitalWrite(Relais_Led_BTN, HIGH);
        if (Cpt_Button == 1)
        {
          Rouge = 0;
          SetAllLed(255, 0, 0);
          delay(300);
          SetAllLed(255, 0, 0);
          delay(300);
          SetAllLed(255, 0, 0);
          delay(300);
          EEPROM.write(1, 0);
          EEPROM.commit();
        }
        if (Cpt_Button == 2)
        {
          Vert = 0;
          SetAllLed(0, 255, 0);
          delay(300);
          SetAllLed(0, 255, 0);
          delay(300);
          SetAllLed(0, 255, 0);
          delay(300);
          EEPROM.write(2, 0);
          EEPROM.commit();
        }
        if (Cpt_Button == 3)
        {
          Bleu = 0;
          SetAllLed(0, 0, 255);
          delay(300);
          SetAllLed(0, 0, 255);
          delay(300);
          SetAllLed(0, 0, 255);
          delay(300);
          EEPROM.write(3, 0);
          EEPROM.commit();
        }
        delay(600);
        digitalWrite(Relais_Led_BTN, LOW);
        FastLED.clear(true);
        delay(600);
        digitalWrite(Relais_Led_BTN, HIGH);
        if (Cpt_Button == 1)
        {
          Rouge = 0;
          SetAllLed(255, 0, 0);
          delay(300);
          SetAllLed(255, 0, 0);
          delay(300);
          SetAllLed(255, 0, 0);
          delay(300);
          EEPROM.write(1, 0);
          EEPROM.commit();
        }
        if (Cpt_Button == 2)
        {
          Vert = 0;
          SetAllLed(0, 255, 0);
          delay(300);
          SetAllLed(0, 255, 0);
          delay(300);
          SetAllLed(0, 255, 0);
          delay(300);
          EEPROM.write(2, 0);
          EEPROM.commit();
        }
        if (Cpt_Button == 3)
        {
          Bleu = 0;
          SetAllLed(0, 0, 255);
          delay(300);
          SetAllLed(0, 0, 255);
          delay(300);
          SetAllLed(0, 0, 255);
          delay(300);
          EEPROM.write(3, 0);
          EEPROM.commit();
        }
        delay(300);
        digitalWrite(Relais_Led_BTN, LOW);
        FastLED.clear(true);
        SetAllLed(0, 0, 0);
        delay(300);
        digitalWrite(Relais_Led_BTN, HIGH);
        delay(300);
        digitalWrite(Relais_Led_BTN, LOW);
        delay(300);
        digitalWrite(Relais_Led_BTN, HIGH);
        if (Cpt_Button == 1)
        {
          Rouge = 0;
          SetAllLed(255, 0, 0);
          delay(300);
          SetAllLed(255, 0, 0);
          delay(300);
          SetAllLed(255, 0, 0);
          delay(300);
          EEPROM.write(1, 0);
          EEPROM.commit();
        }
        if (Cpt_Button == 2)
        {
          Vert = 0;
          SetAllLed(0, 255, 0);
          delay(300);
          SetAllLed(0, 255, 0);
          delay(300);
          SetAllLed(0, 255, 0);
          delay(300);
          EEPROM.write(2, 0);
          EEPROM.commit();
        }
        if (Cpt_Button == 3)
        {
          Bleu = 0;
          SetAllLed(0, 0, 255);
          delay(300);
          SetAllLed(0, 0, 255);
          delay(300);
          SetAllLed(0, 0, 255);
          delay(300);
          EEPROM.write(3, 0);
          EEPROM.commit();
        }
      }

      while (digitalRead(Button) == HIGH)
      { // incrémente tour a tout la luminosité des 3 couleurs
        if (Cpt_Button == 1)
        {
          Rouge = Rouge + 31;
          if (Rouge > 255)
          {
            Rouge = 0;
          }
          EEPROM.write(1, Rouge);
          EEPROM.commit();
        }

        if (Cpt_Button == 2)
        {
          Vert = Vert + 31;
          if (Vert > 255)
          {
            Vert = 0;
          }
          EEPROM.write(2, Vert);
          EEPROM.commit();
        }

        if (Cpt_Button == 3)
        {
          Bleu = Bleu + 31;
          if (Bleu > 255)
          {
            Bleu = 0;
          }
          EEPROM.write(3, Bleu);
          EEPROM.commit();
        }
        SetAllLed(Rouge, Vert, Bleu);
        delay(1200);
      }
    }
    //----------------------------------------------------------------------------------------------------END BTN PRESED ?-------------------//
    //----------------------------------------------------------------------------------------------------MODE LED-------------------//

    if (mode_color == 1) // MODE 1 - STATIC
    {
      SetAllLed(Rouge, Vert, Bleu);
    }

    if (mode_color == 2) // MODE 2 - ALLER -> Puis OFF ->
    {
      for (int i = 0; i <= NUM_LEDS - 1; i++)
      {
        leds[i] = CRGB((Rouge), (Vert), (Bleu));
        FastLED.show();
        delayMicroseconds(5);
        delay(60);
      }
      for (int i = 0; i <= NUM_LEDS - 1; i++)
      {
        leds[i] = CRGB(0, 0, 0);
        FastLED.show();       // Utilisation de l'algo FastLed
        delayMicroseconds(5); // delay de 5  us
        delay(60);            // delay de 60 ms
      }
    }

    if (mode_color == 3) // MODE 3 - jsp ?
    {
      for (int i = 0; i <= NUM_LEDS - 1; i++)
      {
        FastLED.clear(true); // better than set all led
        leds[i] = CRGB((Rouge), (Vert), (Bleu));
        leds[i + 1] = CRGB((Rouge), (Vert), (Bleu));
        leds[i + 2] = CRGB((Rouge), (Vert), (Bleu));
        FastLED.show();       // Utilisation de l'algo FastLed
        delayMicroseconds(5); // delay de 5  us
        delay(80);            // delay de 60 ms
      }
    }

    if (mode_color == 4) // MODE 4 - luminosité --- puis +++ "rapide"
    {
      SetAllLed(Rouge, Vert, Bleu);
      for (int i = 0; i <= 255; i++)
      {
        FastLED.setBrightness(255 - i); // delay de 60 ms
        FastLED.show();
        delay(5);
      }
      for (int i = 0; i <= 255; i++)
      {
        FastLED.setBrightness(i); // delay de 60 ms
        FastLED.show();
        delay(5);
      }
    }

    if (mode_color == 5) // MODE 5 - luminosité --- puis +++ mais "lent"
    {
      SetAllLed(Rouge, Vert, Bleu);
      for (int i = 0; i <= 255; i++)
      {
        FastLED.setBrightness(255 - i); // delay de 60 ms
        FastLED.show();
        delay(10);
      }
      for (int i = 0; i <= 255; i++)
      {
        FastLED.setBrightness(i); // delay de 60 ms
        FastLED.show();
        delay(10);
      }
    }
    //----------------------------------------------------------------------------------------------------END MODE LED-------------------//

    vTaskDelay(pdMS_TO_TICKS(50)); // delay task
  }
}

void ReadVoltage(void *pvParameters)
{
  int VoltageRead;
  float Voltage_Steering_Wheels_Commande;
  float Voltage_Steering_Wheels_5V;
  const char Relais_ST_HAUT = PIN_RELAIS_ACTION_BOUTONS_VOLANT_HAUT; // Relais btn haut volant
  const char Relais_ST_BAS = PIN_RELAIS_ACTION_BOUTONS_VOLANT_BAS;   // Relais btn bas volant

  pinMode(Relais_ST_HAUT, OUTPUT);
  pinMode(Relais_ST_BAS, OUTPUT);
  digitalWrite(Relais_ST_BAS, LOW);
  digitalWrite(Relais_ST_HAUT, LOW);

  // Configurer l'ADC - SETUP
  adc1_config_width(ADC_WIDTH_BIT_12);                                 // Configure la largeur de l'ADC à 12 bits
  adc1_config_channel_atten(ADC_PIN_36, adc_atten_t::ADC_ATTEN_DB_11); // Configure le canal 0 avec une atténuation

  while (1) // boucle infinie
  {

    VoltageRead = adc1_get_raw(adc1_channel_t::ADC1_CHANNEL_0);
    Voltage_Steering_Wheels_Commande = (VoltageRead / 4095.0) * 3.3;

    // Serial.print("\nTension commande btn volant : ");
    // Serial.println(Voltage_Steering_Wheels_Commande);
    LogVariable_1.Tension_Bus_Volant = Voltage_Steering_Wheels_Commande;

    if (1.25 < Voltage_Steering_Wheels_Commande && Voltage_Steering_Wheels_Commande < 1.60) // btn haut 280ohm
    {
      Serial.print("ST HAUT PRESSED");
      digitalWrite(Relais_ST_HAUT, HIGH);
      delay(450);
      digitalWrite(Relais_ST_HAUT, LOW);
    }
    if (2.05 < Voltage_Steering_Wheels_Commande && Voltage_Steering_Wheels_Commande < 2.35) // btn bas 450 ohm
    {
      Serial.print("ST BAS PRESSED");
      digitalWrite(Relais_ST_BAS, HIGH);
      delay(450);
      digitalWrite(Relais_ST_BAS, LOW);
    }


    vTaskDelay(pdMS_TO_TICKS(35)); // delay de 50ms
  }
}

void Retro_Motor(void *pvParameters)
{
#define Temps_Mouvement_Retro 2500
  bool Moteur_Retro_Mode_Parking = 0;

  const char Activation_Moteur_Retro = PIN_DECLANCHEMENT_ABAISSEMENT_RETRO;                    // Pin pour l'activation
  const char Relay_1 = PIN_CONTROLE_RELAIS_RETRO_Relay_1;               
  const char Relay_2 = PIN_CONTROLE_RELAIS_RETRO_Relay_2;               
  const char Relay_3 = PIN_CONTROLE_RELAIS_RETRO_Relay_3; 
  const char Relay_4 = PIN_CONTROLE_RELAIS_RETRO_Relay_4;
  // 4 relay

  // Setup
  pinMode(Relay_2, OUTPUT);
  pinMode(Relay_1, OUTPUT);
  pinMode(Relay_4, OUTPUT);
  pinMode(Relay_3, OUTPUT);
  pinMode(Activation_Moteur_Retro, INPUT);

  digitalWrite(Relay_2, LOW);
  digitalWrite(Relay_1, LOW);
  digitalWrite(Relay_4, LOW);
  digitalWrite(Relay_3, LOW);

  while (1)
  {
if (digitalRead(Activation_Moteur_Retro) == HIGH)
{
  vTaskDelay(pdMS_TO_TICKS(2000)); // delay task 10s
      if (digitalRead(Activation_Moteur_Retro) == HIGH && Moteur_Retro_Mode_Parking == 0)
      {
        Serial.print("recule detecter");
        Moteur_Retro_Mode_Parking = 1;

        digitalWrite(Relay_1, HIGH);
        vTaskDelay(Temps_Mouvement_Retro);
        digitalWrite(Relay_1, LOW);
        vTaskDelay(pdMS_TO_TICKS(10000)); // delay task
      }
}

    if (Moteur_Retro_Mode_Parking == 1)
    {
      if (digitalRead(Activation_Moteur_Retro) == LOW)
      {
        digitalWrite(Relay_2, HIGH);
        delay(500);
        digitalWrite(Relay_3, HIGH);
        digitalWrite(Relay_4, HIGH);
        delay(500);
        digitalWrite(Relay_1, HIGH);
        vTaskDelay(Temps_Mouvement_Retro);
        digitalWrite(Relay_1, LOW);
        delay(200);
        digitalWrite(Relay_3, LOW);
        digitalWrite(Relay_4, LOW);
        delay(200);
        digitalWrite(Relay_2, LOW);



        Moteur_Retro_Mode_Parking = 0;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // delay task
  }
}

// Calcul le % d'une PWM
float mesurerPourcentagePWM(int pwmPin)
{
  unsigned long dureeHigh = pulseIn(pwmPin, HIGH, 50000); // Mesure de la durée de l'impulsion HIGH avec un timeout de 0.05 seconde
  unsigned long dureeLow = pulseIn(pwmPin, LOW, 50000);   // Mesure de la durée de l'impulsion LOW avec un timeout de 0.05 seconde

  // Vérifier si des impulsions ont été détectées
  if (dureeHigh == 0 || dureeLow == 0)
  {
    return -1; // Retourne -1 si aucun signal n'est détecté ou si le timeout est atteint
  }

  float periodeTotale = dureeHigh + dureeLow;
  return (dureeHigh / periodeTotale) * 100; // Retourne le pourcentage du PWM
}

// Fonction pour calculer la vitesse basée sur le pourcentage de PWM -> pas encore validé
int calcule_Vitesse_PWM(int pwmPin, float pwmMin, float pwmMax, int vitesseMin, int vitesseMax)
{
  float pourcentagePWM = mesurerPourcentagePWM(pwmPin);

  // Vérifier si le pourcentage est valide
  if (pourcentagePWM == -1 || pourcentagePWM < pwmMin || pourcentagePWM > pwmMax)
  {
    return -1;
  }

  // Normaliser le pourcentage PWM dans l'intervalle [0, 1]
  float pwmNormalise = (pourcentagePWM - pwmMin) / (pwmMax - pwmMin);
  int vitesse = vitesseMin + (int)(pwmNormalise * (vitesseMax - vitesseMin));
  LogVariable_1.Vitesse_PWM = vitesse;

  // Calculer la vitesse en utilisant une interpolation linéaire sur l'intervalle normalisé
  return vitesse;
}

int calcule_Vitesse_Voltage(int analogPin, float voltMin, float voltMax, int vitesseMin, int vitesseMax)
{
  // Lire la valeur analogique du pin
  int valeurAnalogique = analogRead(analogPin);

  // Convertir la valeur analogique en tension (0 - 3.3V)
  float tension = valeurAnalogique * (3.3 / 4095.0);

  // Vérifier si la tension est dans l'intervalle [voltMin, voltMax]
  if (tension < voltMin || tension > voltMax)
  {
    return -1; // Retourner -1 si la tension est hors limites
  }

  // Normaliser la tension dans l'intervalle [0, 1]
  float tensionNormalisee = (tension - voltMin) / (voltMax - voltMin);

  // Calculer la vitesse en utilisant une interpolation linéaire sur l'intervalle normalisé
  int vitesse = vitesseMin + (int)(tensionNormalisee * (vitesseMax - vitesseMin));
  LogVariable_1.Vitesse_Tension = vitesse;

  return vitesse;
}

void Calcule_Vitesse(void *pvParameters)
{
  while (1)
  {
    int A = calcule_Vitesse_PWM(13, 0, 100, 0, 250);
    int B = calcule_Vitesse_Voltage(14, 0, 3.4, 0, 300); // Fonction validé

    vTaskDelay(pdMS_TO_TICKS(50)); // delay de 35ms
  }
}

void Gestion_Bluetooth(void *pvParameters)
{
  SerialBT.begin("ESP32-MZ-BT"); // Initialiser le Bluetooth avec le nom du point d'accès
  Serial.println("Le point d'accès Bluetooth 'ESP32-MZ-BT' est actif!");
  while (1)
  {
    if (SerialBT.connected())
    {
      LogVariable_1.Connexion_BT = 1;
    }
    else
    {
      LogVariable_1.Connexion_BT = 0;
    }

    if (SerialBT.available())
    {
      String data = SerialBT.readString(); // Lire les données reçues
      Serial.print("\nReçu en Bluetooth: ");
      Serial.println(data);     // Afficher les données sur le port série
      SerialBT.print("Echo: "); // Envoyer un écho de ces données de retour au dispositif connecté
      SerialBT.println(data);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void Gestion_LOGs(void *pvParameters)
{
  while (1)
  {
    Serial.print("\nLOGS : \n");
    Serial.print("F_CPU : ");
    Serial.print(getCpuFrequencyMhz());
    Serial.print(", Tension_Bus_Volant : ");
    Serial.print(LogVariable_1.Tension_Bus_Volant);
    Serial.print(", Vitesse (Tension)/(PWM) : ");
    Serial.print(LogVariable_1.Vitesse_Tension);
    Serial.print(" / ");
    Serial.print(LogVariable_1.Vitesse_PWM);
    Serial.print(", Connexion BT : ");
    if (LogVariable_1.Connexion_BT)
    {
      Serial.print("Oui");
    }
    else
    {
      Serial.print("Non");
    }
    Serial.print("\n Charge CPU : ");
    Serial.print(LogVariable_1.Charge_CPU);



    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}




//---------------- ----------------------------------------------------------------------------------------------------------//
// SETUP
//--------------------------------------------------------------------------------------------------------------------------//
void setup()
{

  esp_register_freertos_idle_hook(myIdleHook); // Enregistrez votre hook

  // FastLED.addLeds<LED_TYPE,LED_PIN,COLOR_ORDER>(leds,NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  Serial.begin(115200);
  Serial.println("BOOT...\n");

  for (int y = 0; y <= 2; y++)
  {
    for (int i = 0; i <= 255; i++) // led brightness rise down -> 0
    {
      FastLED.setBrightness(255 - i);
      FastLED.show();
      delay(1);
    }
    for (int i = 0; i <= 255; i++) // led brightness rise up -> 255
    {
      FastLED.setBrightness(i);
      FastLED.show();
      delay(1);
    }
  }
  delay(20);
  FastLED.clear(true);

  // Créer une tâche qui s'exécute sur le noyau 1.
  xTaskCreatePinnedToCore(
      ReadVoltage,   // Pour controle des boutons au volant
      "ReadVoltage", /* Nom de la tâche */
      10000,         /* Taille de la pile pour la tâche */
      NULL,          /* Paramètre passé à la tâche */
      4,             /* Priorité de la tâche */
      NULL,          /* Handle de la tâche */
      0              /* Le cœur sur lequel la tâche doit être exécutée */
  );

  xTaskCreatePinnedToCore(
      Gestion_Led,   /* La fonction à exécuter */
      "Gestion_Led", /* Nom de la tâche */
      10000,         /* Taille de la pile pour la tâche */
      NULL,          /* Paramètre passé à la tâche */
      1,             /* Priorité de la tâche */
      NULL,          /* Handle de la tâche */
      1              /* Le cœur sur lequel la tâche doit être exécutée */
  );

  xTaskCreatePinnedToCore(
      Retro_Motor,   // Pour controle des boutons au volant
      "Retro_Motor", /* Nom de la tâche */
      10000,         /* Taille de la pile pour la tâche */
      NULL,          /* Paramètre passé à la tâche */
      3,             /* Priorité de la tâche */
      NULL,          /* Handle de la tâche */
      1              /* Le cœur sur lequel la tâche doit être exécutée */
  );

  xTaskCreatePinnedToCore(
      Calcule_Vitesse,   // Pour controle des boutons au volant
      "Calcule_Vitesse", /* Nom de la tâche */
      10000,             /* Taille de la pile pour la tâche */
      NULL,              /* Paramètre passé à la tâche */
      2,                 /* Priorité de la tâche */
      NULL,              /* Handle de la tâche */
      1                  /* Le cœur sur lequel la tâche doit être exécutée */
  );

  xTaskCreatePinnedToCore(
      Gestion_Bluetooth,   // Pour controle des boutons au volant
      "Gestion_Bluetooth", /* Nom de la tâche */
      10000,               /* Taille de la pile pour la tâche */
      NULL,                /* Paramètre passé à la tâche */
      1,                   /* Priorité de la tâche */
      NULL,                /* Handle de la tâche */
      1                    /* Le cœur sur lequel la tâche doit être exécutée */
  );
  xTaskCreatePinnedToCore(
      Gestion_LOGs,   // Pour controle des boutons au volant
      "Gestion_LOGs", /* Nom de la tâche */
      10000,          /* Taille de la pile pour la tâche */
      NULL,           /* Paramètre passé à la tâche */
      1,              /* Priorité de la tâche */
      NULL,           /* Handle de la tâche */
      0               /* Le cœur sur lequel la tâche doit être exécutée */
  );

}

void loop(void)
{
  // printCpuFrequency();
   //setCpuFreq(80); //240 , 160, 80

}
