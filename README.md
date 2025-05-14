# LifeGG

#Ce projet implémente un portique motorisé :

    Il s'ouvre automatiquement lorsqu'une personne est détectée par un capteur PIR.

    Il se ferme automatiquement après 10 secondes d'inactivité.

    L'ouverture et la fermeture du portique durent chacune 10 secondes.

    Des LEDs (L1 et L2) indiquent visuellement l’état du portique.

    Une interface UART (TeraTerm) affiche les états du PIR et du portique.

 #Matériel utilisé :

    Nucleo-L152RE (STM32L152 microcontrôleur)

    Base Shield Seeedstudio

    Capteur PIR Grove Mini v1.0 (détection de mouvement)

    Moteur DC

    LEDs L1 (PB2) & L2 (PB10) pour indiquer ouverture/fermeture

    TeraTerm (UART 115200 bauds) pour le debug

 #Fonctionnement logique :

    PIR détecte présence (PA10 = 1) :

        Le moteur démarre en ouverture 10s.

        LED L1 (PB2) s’allume.

        Tant qu’il y a détection, il reste ouvert.

    Après 10s sans détection (PIR = 0) :

        Le moteur démarre la fermeture 10s.

        LED L2 (PB10) s’allume.

        Le portique revient à l’état fermé.

    Réouverture possible à tout moment si PIR redétecte.

 #Architecture du code :

    main.c : boucle principale avec la logique portique.

    Fonctions :

        set_motor_direction(uint8_t open) : démarre PWM + configure sens moteur.

        stop_motor() : arrête le moteur.

    HAL_TIM_PWM_Start / Stop pour moteur.

    HAL_GPIO_WritePin pour LEDs et DIR.

    MX_TIM3_Init() configure TIM3 CH1 en PWM.

    MX_USART2_UART_Init() configure l’UART pour le debug.

  #À noter :

    TIM3_CH2 n’est pas utilisé.

    Le moteur tourne uniquement via TIM3_CH1.

    La direction est gérée par PB5 en simple GPIO.

    LEDs L1 / L2 permettent une visualisation directe.

    UART affiche : PIR status, état du portique.

  
