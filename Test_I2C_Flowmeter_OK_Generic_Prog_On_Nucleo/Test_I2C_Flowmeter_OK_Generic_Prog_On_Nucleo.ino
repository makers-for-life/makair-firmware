

#include <Wire.h>

//HardwareSerial Serial (PA3, PA2);  //Liaison UART2 suivi du convertisseur USB

// CRC calculation as per:
// https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/5_Mass_Flow_Meters/Sensirion_Mass_Flow_Meters_CRC_Calculation_V1.pdf

#define POLYNOMIAL 0x31     //P(x)=x^8+x^5+x^4+1 = 100110001

uint8_t CRC_prim (uint8_t x, uint8_t crc) {
  crc ^= x;
  for (uint8_t bit = 8; bit > 0; --bit) {
    if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
    else crc = (crc << 1);
  }
  return crc;
}

#define sfm3300i2c 0x40

void setup() {

  //TwoWire Wire(PB9,PB8);      //Define SDA and SCL Pins, why is it not necessary on nucleo and needed for generic µcontroller ?
  //Wire.begin(PB9,PB8);      //I2C init (SDA, SCL), not sure it works
  Wire.setSDA(PB9);         
  Wire.setSCL(PB8);
  Wire.begin();
  //I2C_InitStructure.I2C_ClockSpeed = 10000;
  //Wire.setClock(100000);      //100K = default, 10K gives default (100K), 400K works
 
  Serial.begin(115200);
  delay(500); // let Serial console settle

  // soft reset
  Wire.beginTransmission(sfm3300i2c);
  Wire.write(0x20);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

#if 1
  Wire.beginTransmission(sfm3300i2c);
  Wire.write(0x31);  // read Serial number
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
    Serial.println(sn);
  } else {
    Serial.println("Serial number - i2c read error");
  }
#endif

  // start continuous measurement
  Wire.beginTransmission(sfm3300i2c);
  Wire.write(0x10);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(100);
  /* // discard the first chunk of data that is always 0xFF
  Wire.requestFrom(sfm3300i2c,3);
  Wire.read();
  Wire.read();
  Wire.read();
  */

  Serial.println("Flow\tVolume");
}


const unsigned mms = 10; // measurement interval in ms
const unsigned dms = 1000; // display every XXX ms

unsigned long mt_prev = millis(); // last measurement time-stamp
float flow; // current flow value in slm
float vol;  // current volume value in (standard) cubic centimeters
bool flow_sign; // flow sign
bool flow_sp; // previous value of flow sign
bool crc_error; // flag

void display_flow_volume(bool force_d = false) {
  if (5 < abs(vol) || force_d) { // for convenience let's display only significant volumes (>5ml)
    Serial.print(flow);
    Serial.print("\t");
    Serial.print(vol);
    Serial.println(crc_error?" CRC error":"");
  }
}

void SFM_measure() {
  if (3 == Wire.requestFrom(sfm3300i2c, 3)) {
    uint8_t crc = 0;
    uint16_t a = Wire.read();
    crc = CRC_prim (a, crc);
    uint8_t  b = Wire.read();
    crc = CRC_prim (b, crc);
    uint8_t  c = Wire.read();
    unsigned long mt = millis(); // measurement time-stamp
    if (crc_error = (crc != c)) // report CRC error
      return;
    a = (a<<8) | b;
    float new_flow = ((float)a - 32768) / 120;
    // an additional functionality for convenience of experimenting with the sensor
    flow_sign = 0 < new_flow;
    if (flow_sp != flow_sign) { // once the flow changed direction
      flow_sp = flow_sign;
      display_flow_volume();    // display last measurements
      vol = 0;                  // reset the volume
    }
    flow = new_flow;
    unsigned long mt_delta = mt - mt_prev; // time interval of the current measurement
    mt_prev = mt;
    vol += flow/60*mt_delta; // flow measured in slm; volume calculated in (s)cc
    // /60 --> convert to liters per second
    // *1000 --> convert liters to cubic centimeters
    // /1000 --> we count time in milliseconds
    // *mt_delta --> current measurement time delta in milliseconds
  } else {
    // report i2c read error
  }
}


unsigned long ms_prev = millis(); // timer for measurements "soft interrupts"
unsigned long ms_display = millis(); // timer for display "soft interrupts"


void loop() {
  unsigned long ms_curr = millis();
  
  if (ms_curr - ms_prev >= mms) { // "soft interrupt" every mms milliseconds
    ms_prev = ms_curr;
    SFM_measure();
  }

  if (ms_curr - ms_display >= dms) { // "soft interrupt" every dms milliseconds
    ms_display = ms_curr;
    display_flow_volume(true);
  }
}