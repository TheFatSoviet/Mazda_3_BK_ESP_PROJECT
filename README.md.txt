# Contrôleur Bluetooth et Gestion ADC pour ESP32

## Auteur
Ce projet a été développé par **Rubyan**.

## Description
Ce projet est conçu pour l'ESP32 et utilise ses capacités Bluetooth et ADC (Convertisseur Analogique-Numérique) pour contrôler divers composants électroniques, tels que des LEDs et des relais, tout en recueillant des données analogiques via les broches ADC. Le programme permet également de gérer ces éléments à distance via une connexion Bluetooth. Le code est développé sur PlatformIO.

## Broches Utilisées sur l'ESP32

### Broches pour les Relais et les Boutons
- **PIN_RELAIS_ACTION_BOUTONS_VOLANT_HAUT (GPIO 26)** : Contrôle le relais pour le bouton haut du volant.
- **PIN_RELAIS_ACTION_BOUTONS_VOLANT_BAS (GPIO 25)** : Contrôle le relais pour le bouton bas du volant.
- **PIN_BOUTON_INPUT_GESTION_LED (GPIO 32)** : Entrée pour le bouton de gestion des LEDs.
- **PIN_RELAIS_GESTION_LED (GPIO 33)** : Contrôle le relais pour la gestion des LEDs.

### Broches pour le Contrôle des Rétroviseurs
- **PIN_DECLANCHEMENT_ABAISSEMENT_RETRO (GPIO 22)** : Déclenche l'abaissement du rétroviseur (feu de recul).
- **PIN_CONTROLE_RELAIS_RETRO_Relay_1 (GPIO 5)** : Contrôle le relais pour le rétroviseur (câble jaune).
- **PIN_CONTROLE_RELAIS_RETRO_Relay_2 (GPIO 18)** : Contrôle le relais pour le rétroviseur (câble blanc).
- **PIN_CONTROLE_RELAIS_RETRO_Relay_3 (GPIO 19)** : Contrôle le relais pour le rétroviseur (câble rouge).
- **PIN_CONTROLE_RELAIS_RETRO_Relay_4 (GPIO 21)** : Contrôle le relais pour le rétroviseur (câble noir).

### Broches ADC
- **ADC_PIN_36 (GPIO 36)** : Utilisé pour mesurer la tension positive liée aux boutons du volant.

### LED
- **LED_PIN (GPIO 2)** : Contrôle les LEDs WS2812.

## Fonctions Principales
- **setup()** : Initialise les composants et configure les broches. Prépare la communication Bluetooth, configure l'ADC et crée les tâches FreeRTOS.
- **loop()** : Boucle principale qui reste vide, toutes les fonctions étant gérées par les tâches FreeRTOS.

### Tâches FreeRTOS
- **ReadVoltage(void \*pvParameters)** : Lit la tension des boutons du volant via l'ADC et active les relais correspondants.
- **Retro_Motor(void \*pvParameters)** : Gère le mouvement des rétroviseurs en fonction de l'état des relais.
- **Gestion_Led(void \*pvParameters)** : Gère l'allumage et le mode des LEDs en fonction de l'entrée du bouton.
- **Gestion_Bluetooth(void \*pvParameters)** : Gère la communication Bluetooth avec les appareils connectés.
- **Gestion_LOGs(void \*pvParameters)** : Enregistre et affiche les données de diagnostic, comme la charge CPU et l'état des connexions.

### Fonctions Utilitaires
- **SetAllLed(char R, char G, char B)** : Allume toutes les LEDs avec la couleur spécifiée.
- **mesurerPourcentagePWM(int pwmPin)** : Mesure le pourcentage du signal PWM sur une broche spécifiée.
- **calcule_Vitesse_PWM(int pwmPin, float pwmMin, float pwmMax, int vitesseMin, int vitesseMax)** : Calcule la vitesse en fonction du signal PWM mesuré.
- **calcule_Vitesse_Voltage(int analogPin, float voltMin, float voltMax, int vitesseMin, int vitesseMax)** : Calcule la vitesse en fonction de la tension mesurée.

## Environnement de Développement
Le projet est configuré et développé sur **PlatformIO**, un environnement de développement intégré (IDE) qui permet une gestion simplifiée des bibliothèques et des configurations spécifiques à l'ESP32.
