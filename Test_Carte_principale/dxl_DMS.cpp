#include "dxl_DMS.h"
#include <Arduino.h>


byte dxl_Header[4]          = {0xFF,0xFF,0xFD,0x00};
byte dxl_ID                 = 0x01;
byte dxl_Length[2]          = {0};  //Number of param + 3(for CRC and Instr bytes) 
byte dxl_Instruction        = 0x00;
byte dxl_Param[5]           = {0};
byte dxl_CRC[2]             = {0};
byte dxl_Frame_To_Send[20]  = {dxl_Header[0],dxl_Header[1],dxl_Header[2],dxl_Header[3],dxl_ID};
byte dxl_Frame_Received[20] = {};




//For now it empty the buffer and print the frame;  TBD : check if no error + cut the frame correctly when 2 instructions are sent one after another
void dxl_Read_Data(){         
  Serial6.enableHalfDuplexRx();
  int j = 0;
  delay(3);               //3ms = empirique; laisse le temps à la trame d'arriver et d'être traitée. Le servomoteur est programmé pour envoyer une trame de réponse 500µs après réception de la trame de commande (ce temps est paramétrable est peu être réduit)
  while (Serial6.available()){
    byte incomingByte = Serial6.read();
    dxl_Frame_Received[j]=incomingByte;
    j++;
  }
  Serial2.write(dxl_Frame_Received,j);
}


void dxl_Write_Frame(){
  unsigned short CRC_Calc;
  int dxl_Frame_size  = 0;
  dxl_Frame_size = 7 + dxl_Length[0];  // 7 octets + Nb param
  dxl_Frame_To_Send[5] = dxl_Length[0];
  dxl_Frame_To_Send[6] = dxl_Length[1];
  dxl_Frame_To_Send[7] = dxl_Instruction;
  for (int i=0;i<dxl_Length[0]-3;i++){dxl_Frame_To_Send[8+i] = dxl_Param[i];}
  CRC_Calc = update_crc(0,dxl_Frame_To_Send,dxl_Frame_size-2);
  dxl_CRC[0] = CRC_Calc & 0x00FF;         //poids faible
  dxl_CRC[1] = (CRC_Calc >> 8) & 0x00FF;  //poids fort
  dxl_Frame_To_Send[dxl_Frame_size-2] = dxl_CRC[0];
  dxl_Frame_To_Send[dxl_Frame_size-1] = dxl_CRC[1];
  Serial6.write(dxl_Frame_To_Send,dxl_Frame_size);  
  //dxl_Read_Data();                  //TBD: check CRC and No error
}

void dxl_OperatingMode(char Mode[3]){
  dxl_Instruction                           = 0x03;    //write instruction
  dxl_Param[0]                              = 0x0B;   //Operating mode register, Attention le couple (torque) doit être désactivé
  dxl_Param[1]                              = 0x00; 
  if (strcmp(Mode, "PWM") ==0){dxl_Param[2] = 0x10;}
  if (strcmp(Mode, "Pos") ==0){dxl_Param[2] = 0x03;}
  dxl_Length[0]                             = 0x06;
  dxl_Length[1]                             = 0x00;
  dxl_Write_Frame();
}


void dxl_Position(uint32_t Pos_Pulse){  //entre 0 et 4095 sur quatres octets
  dxl_Instruction          = 0x03;     //write instruction
  dxl_Param[0]             = 0x74;     //Goal Position register
  dxl_Param[1]             = 0x00;
  dxl_Param[5]             = Pos_Pulse >> 24;  
  dxl_Param[4]             = Pos_Pulse >> 16;  
  dxl_Param[3]             = Pos_Pulse >> 8;  
  dxl_Param[2]             = Pos_Pulse & 0xFF;
  dxl_Length[0]            = 0x09;
  dxl_Length[1]            = 0x00;
  dxl_Write_Frame();
}


void dxl_PWM(int16_t PWM){  //entre -885 et 885 sur deux octets
  dxl_Instruction          = 0x03;     //write instruction
  dxl_Param[0]             = 0x64;     //Goal PWM register
  dxl_Param[1]             = 0x00;
  dxl_Param[3]             = PWM >> 8;  
  dxl_Param[2]             = PWM & 0xFF;
  dxl_Length[0]            = 0x07;
  dxl_Length[1]            = 0x00;
  dxl_Write_Frame();
}

void dxl_LED(bool Power){
  dxl_Instruction               = 0x03;    //write instruction
  dxl_Param[0]                  = 0x41;   //LED register
  dxl_Param[1]                  = 0x00;  
  if (Power==true){dxl_Param[2] = 0x01;}
  if (Power==false){dxl_Param[2]= 0x00;}
  dxl_Length[0]                 = 0x06;
  dxl_Length[1]                 = 0x00;
  dxl_Write_Frame();
}

void dxl_EnableTorque(bool Enable){
  dxl_Instruction               = 0x03;    //write instruction
  dxl_Param[0]                  = 0x40;   //Torque register
  dxl_Param[1]                  = 0x00;  
  if (Enable==true){dxl_Param[2]= 0x01;}
  if (Enable==false){dxl_Param[2]=0x00;}
  dxl_Length[0]=0x06;
  dxl_Length[1]=0x00;
  dxl_Write_Frame();
}

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}