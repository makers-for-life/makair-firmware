#include <cstdint>

void dxl_EnableTorque(bool Enable);

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);

void dxl_LED(bool Power);

void dxl_PWM(int16_t PWM);
void dxl_Position(uint32_t Pos_Pulse);  //entre 0 et 4095 pour 1 tour, 2048 = origine, incrémentation = sens trigo, décrementer = sens horaire 
void dxl_OperatingMode(char Mode[3]);