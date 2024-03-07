#include <Wire.h>                     //liaison I²C
#include "dxl_DMS.h"

HardwareSerial Serial6 (PC6);        //liaison TTL avec servomoteurs (3fils: GND, 12V, signal)
//HardwareSerial Serial6(PC6, HALF_DUPLEX_ENABLED); ne fonctionne pas

HardwareSerial Serial2 (PA3, PA2);  //Liaison UART2 suivi du convertisseur USB

#define BP_S12        PB12              // Bouton poussoir sur carte de Dev
#define Enable_RPI    PD2               // Allume/Eteins la Raspberry
#define LEDV          PB14
#define LEDR          PC8
#define BUZZER        PB7
#define Blower_PWM    PA10               // Consigne PWM de la turbine (entre 0 et 254 théorique mais plutot prendre entre 0 et 250)
#define sfm3300i2c    0x40              //Adresse I²C par défaut du capteur de débit SFM3300
#define Pressure_Ana  PA1               //Entrée analogique du capteur de pression
#define LED_V         PB14
#define LED_R         PC8

bool OneTime  = false;
int State     = 0;

void setup() {
  Serial2.begin(115200);
  Serial6.begin(1000000);
  Serial2.println("Bonjour");
  pinMode(BP_S12, INPUT);
  pinMode(Pressure_Ana, INPUT);
  pinMode(Blower_PWM, OUTPUT);
  pinMode(Enable_RPI, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_V, OUTPUT);
  pinMode(LED_R, OUTPUT);
  analogWriteFrequency(20000);        //Blower pwm frequency
  //analogWriteFrequency(5000);       //uncomment to hear the buzzer - NB : Blower will not work anymore
  digitalWrite(Enable_RPI, HIGH);
  
  Wire.setSDA(PB9);         
  Wire.setSCL(PB8);
  Wire.begin();
  delay(500); // let serial console settle



  // soft reset
  Wire.beginTransmission(sfm3300i2c);
  Wire.write(0x20);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  #if 1
    digitalWrite(Enable_RPI, LOW);        //power ON RPI
    Wire.beginTransmission(sfm3300i2c);
    Wire.write(0x31);  // read serial number
    Wire.write(0xAE);  // command 0x31AE
    Wire.endTransmission();
    if (6 == Wire.requestFrom(sfm3300i2c,6)) {
      uint32_t sn = 0;
      sn = Wire.read(); sn <<= 8;
      sn += Wire.read(); sn <<= 8;
      Wire.read(); // CRC - omitting for now
      sn += Wire.read(); sn <<= 8;
      sn += Wire.read();
      Wire.read(); // CRC - omitting for now
      Serial2.println(sn);
    } else {
      Serial2.println("serial number - i2c read error");
    }
  #endif
}

void loop() {
 
  if((digitalRead(BP_S12)==false)& OneTime==false){        // Si bouton et pas de bouton à la scrutation précédente
    OneTime=true;
    analogWrite(Blower_PWM,100);
    digitalWrite(LED_V, LOW);
    digitalWrite(LED_R, LOW);
    Serial2.println("BP_OFF");
    //analogWrite(BUZZER,255);        //fonctionne mais suite à changement de microcontroleur il y un soucis (restera commenté jusqu'au premier de série)
    // chaque case dans le switch n'est éxécuté qu'uune fois, à l'appui sur le bouton
    switch (State){
      case 0:
        /* // dans le cas ou l'on souhaite un controle en position du servomoteur
        dxl_OperatingMode("Pos");
        dxl_Position(4095);
        */
        break;
      case 1:         //init + tester si communication OK (A dev)
        dxl_OperatingMode("PWM");
        dxl_LED(true);
        dxl_EnableTorque(true);
        break;
      case 2:
        dxl_PWM(-500);
        break;
      case 3:
        dxl_PWM(880);
        break;
      case 4:
        dxl_PWM(0);
        dxl_LED(false);
        dxl_EnableTorque(false);
        break;
      case 5: 
        //test de la remontée du capteur de pression   
        break;
      case 6:
        //test remontée information capteur de débit
        break;
      default:
        State=0;
        break; 
    }
    State++; 
  }
  else{     // Si pas de bouton et bouton à la scrutation précédente
    if ((digitalRead(BP_S12)==true)& OneTime==true){
      Serial2.println("BP_ON");
      OneTime=false;
      analogWrite(Blower_PWM,50);
      //analogWrite(BUZZER,10);
      digitalWrite(LED_V, HIGH);
      digitalWrite(LED_R, HIGH);
    }
  }

  //test de la remontée du capteur de pression 
  if (State==5){
    Serial2.println(analogRead(Pressure_Ana));   
    delay(500);
  }                   
  

  if (State==6){
     // start continuous measurement (command 0x1000)
    Wire.write(0x10); 
    Wire.write(0x00);
    Wire.endTransmission();
    delay(500);
    
    if (2 == Wire.requestFrom(sfm3300i2c, 2)) { // just keep reading SLM (Standard Liter per Minute)
      uint16_t a = Wire.read(); // only two bytes need to be read
      uint8_t  b = Wire.read(); // if we don't care about CRC
      a = (a<<8) | b;
      float flow = ((float)a - 32768) / 120;
      Serial2.println(flow);
    }
  }
  
}

extern "C" void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 100;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
     */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
}

