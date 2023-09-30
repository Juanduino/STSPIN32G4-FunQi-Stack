
#include "./MT6835_STM32G4.h"
#include "common/foc_utils.h"



MT6835_STM32G4::MT6835_STM32G4(int nCS) : nCS(nCS) {
    // nix
};

MT6835_STM32G4::~MT6835_STM32G4() {
    // nix
};



void MT6835_STM32G4::init() {
    
    if (nCS >= 0) {
        pinMode(nCS, OUTPUT);
        digitalWrite(nCS, HIGH);
    }

 SPI.begin(); //Initialize the SPI_1 port.
 SPI.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
 SPI.setDataMode(SPI_MODE3); //Set the  SPI_2 data mode 0
 SPI.setClockDivider(SPI_CLOCK_DIV8);      // Slow speed (72 / 16 = 4.5 MHz SPI_1 speed)

};




float MT6835_STM32G4::getCurrentAngle(){
    return getRaw16bitTIM3() / (float)65536 * _2PI;
};



uint16_t MT6835_STM32G4::getRaw16bitTIM3(){

return TIM3->CNT;

}


uint32_t MT6835_STM32G4::getRaw21bitMT6835(){

  uint8_t angle_buffer[3];

  digitalWrite(nCS, LOW); // manually take CSN low for SPI_1 transmission
  SPI.transfer(0xA0, SPI_CONTINUE); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  SPI.transfer(0x003, SPI_CONTINUE); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  angle_buffer[0] = SPI.transfer(0x00, SPI_CONTINUE); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  angle_buffer[1] = SPI.transfer(0x00, SPI_CONTINUE); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  angle_buffer[2] = SPI.transfer(0x00, SPI_CONTINUE); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  SPI.transfer(0x00, SPI_LAST); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  digitalWrite(nCS, HIGH); // manually take CSN high between spi transmissions

  /*
  SerialUSB.println("Read Register 0x003:  ");
  SerialUSB.println(data0, BIN);
  SerialUSB.println(data1, BIN);
  SerialUSB.println(angle_buffer[0], BIN);
  SerialUSB.println(angle_buffer[1], BIN);
  SerialUSB.println(angle_buffer[2], BIN);
  SerialUSB.println(data5, BIN);
  */

  uint32_t buff_shift = (angle_buffer[0] << 13) | (angle_buffer[1] << 5) | (angle_buffer[2] >> 3);
  return buff_shift;
    
}


uint32_t MT6835_STM32G4::readRawAngle21(){
    uint8_t data[6]; // transact 48 bits
    data[0] = (MT6835_OP_ANGLE<<4);
    data[1] = MT6835_REG_ANGLE1;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    if (nCS >= 0)
        digitalWrite(nCS, LOW);
    spi->beginTransaction(settings);
    spi->transfer(data, 6);
    spi->endTransaction();
    if (nCS >= 0)
        digitalWrite(nCS, HIGH);
    return (data[2] << 13) | (data[3] << 5) | (data[4] >> 3);
};




bool MT6835_STM32G4::setZeroFromCurrentPosition(){
    
    digitalWrite(nCS, LOW); // manually take CSN low for SPI_1 transmission
    SPI.transfer(MT6835_OP_ZERO, SPI_CONTINUE); 
    SPI.transfer(0x00, SPI_CONTINUE); 
    if (SPI.transfer(0x00, SPI_LAST) == MT6835_WRITE_ACK){
    digitalWrite(nCS, HIGH); // manually take CSN high between spi transmissions    
    return true; }
    else {digitalWrite(nCS, HIGH); // manually take CSN high between spi transmissions
    return false;}

};

uint8_t MT6835_STM32G4::readMT6835_CAL_FREQ(){
    uint8_t CAL_FREQ = 0;
    digitalWrite(nCS, LOW); // manually take CSN low for SPI_1 transmission
    SPI.transfer(0x30, SPI_CONTINUE); 
    SPI.transfer(0x00E, SPI_CONTINUE); 
    CAL_FREQ = SPI.transfer(0x00, SPI_LAST);
    digitalWrite(nCS, HIGH); // manually take CSN high between spi transmissions
    return CAL_FREQ;

};

uint8_t MT6835_STM32G4::writeMT6835_CAL_FREQ(){

    uint8_t WRITE_CAL_FREQ = 0;
    digitalWrite(nCS, LOW); // manually take CSN low for SPI_1 transmission
    SPI.transfer(0x60, SPI_CONTINUE); 
    SPI.transfer(0x00E, SPI_CONTINUE); 
    WRITE_CAL_FREQ = SPI.transfer(0x7F, SPI_LAST);
    digitalWrite(nCS, HIGH); // manually take CSN high between spi transmissions
    return WRITE_CAL_FREQ;

};


uint16_t MT6835_STM32G4::readMT6835_ABZ(){

    uint8_t read_ABZ1 = 0;
    uint8_t read_ABZ2 = 0;
    digitalWrite(nCS, LOW); // manually take CSN low for SPI_1 transmission
    SPI.transfer(0x30, SPI_CONTINUE); 
    SPI.transfer(0x007, SPI_CONTINUE); 
    read_ABZ1 = SPI.transfer(0x00, SPI_LAST);
    digitalWrite(nCS, HIGH); // manually take CSN high between spi transmissions
    Serial.println("ABZ1 ->");
    Serial.println(read_ABZ1, BIN);
    delay(200);
    digitalWrite(nCS, LOW); // manually take CSN low for SPI_1 transmission
    SPI.transfer(0x30, SPI_CONTINUE); 
    SPI.transfer(0x008, SPI_CONTINUE); 
    read_ABZ2 = SPI.transfer(0x00, SPI_LAST);
    digitalWrite(nCS, HIGH); // manually take CSN high between spi transmissions
    Serial.println("ABZ2 ->");
    Serial.println(read_ABZ2, BIN);

    return (uint16_t)read_ABZ1 | read_ABZ2;

};

uint8_t MT6835_STM32G4::setMT6835_ABZ1(){

    uint8_t WRITE_CAL_FREQ = 0;
    digitalWrite(nCS, LOW); // manually take CSN low for SPI_1 transmission
    SPI.transfer(0x60, SPI_CONTINUE); 
    SPI.transfer(0x007, SPI_CONTINUE); 
    WRITE_CAL_FREQ = SPI.transfer(0xFF, SPI_LAST);
    digitalWrite(nCS, HIGH); // manually take CSN high between spi transmissions
    return WRITE_CAL_FREQ;

};

uint8_t MT6835_STM32G4::setMT6835_ABZ2(){

    uint8_t WRITE_CAL_FREQ = 0;
    digitalWrite(nCS, LOW); // manually take CSN low for SPI_1 transmission
    SPI.transfer(0x60, SPI_CONTINUE); 
    SPI.transfer(0x008, SPI_CONTINUE); 
    WRITE_CAL_FREQ = SPI.transfer(0xFC, SPI_LAST);
    digitalWrite(nCS, HIGH); // manually take CSN high between spi transmissions
    return WRITE_CAL_FREQ;
};


/**
 * Wait 6s after calling this method
 */
bool MT6835_STM32G4::writeEEPROM(){

    digitalWrite(nCS, LOW); // manually take CSN low for SPI_1 transmission
    SPI.transfer(0xC0, SPI_CONTINUE); 
    SPI.transfer(0x00, SPI_CONTINUE); 
    if (SPI.transfer(0x00, SPI_LAST) == MT6835_WRITE_ACK){
    digitalWrite(nCS, HIGH); // manually take CSN high between spi transmissions    
    return true; }
    else {digitalWrite(nCS, HIGH); // manually take CSN high between spi transmissions
    return false;}

};





uint8_t MT6835_STM32G4::getBandwidth(){
    MT6835Options5 opts = { .reg = readRegister(MT6835_REG_OPTS5) };
    return opts.bw;
};
void MT6835_STM32G4::setBandwidth(uint8_t bw){
    MT6835Options5 opts = { .reg = readRegister(MT6835_REG_OPTS5) };
    opts.bw = bw;
    writeRegister(MT6835_REG_OPTS5, opts.reg);
};

uint8_t MT6835_STM32G4::getHysteresis(){
    MT6835Options3 opts = { .reg = getOptions3().reg };
    return opts.hyst;
};
void MT6835_STM32G4::setHysteresis(uint8_t hyst){
    MT6835Options3 opts = { .reg = getOptions3().reg };
    opts.hyst = hyst;
    setOptions3(opts);
};

uint8_t MT6835_STM32G4::getRotationDirection(){
    MT6835Options3 opts = { .reg = getOptions3().reg };
    return opts.rot_dir;
};
void MT6835_STM32G4::setRotationDirection(uint8_t dir){
    MT6835Options3 opts = { .reg = getOptions3().reg };
    opts.rot_dir = dir;
    setOptions3(opts);
};


uint16_t MT6835_STM32G4::getABZResolution(){
    uint8_t hi = readRegister(MT6835_REG_ABZ_RES1);
    MT6835ABZRes lo = {
			.reg = readRegister(MT6835_REG_ABZ_RES2)
	};
    return (hi << 6) | lo.abz_res_low;
};
void MT6835_STM32G4::setABZResolution(uint16_t res){
     uint8_t hi = (res >> 2);
    MT6835ABZRes lo = {
			.reg = readRegister(MT6835_REG_ABZ_RES2)
	};
    lo.abz_res_low = res & 0x3F;
    writeRegister(MT6835_REG_ABZ_RES1, hi);
    writeRegister(MT6835_REG_ABZ_RES2, lo.reg);
};



bool MT6835_STM32G4::isABZEnabled(){
    MT6835ABZRes lo = {
			.reg = readRegister(MT6835_REG_ABZ_RES2)
	};
    return lo.abz_off==0;
};
void MT6835_STM32G4::setABZEnabled(bool enabled){
    MT6835ABZRes lo = {
			.reg = readRegister(MT6835_REG_ABZ_RES2)
	};
    lo.abz_off = enabled?0:1;
    writeRegister(MT6835_REG_ABZ_RES2, lo.reg);
};



bool MT6835_STM32G4::isABSwapped(){
    MT6835ABZRes lo = {
			.reg = readRegister(MT6835_REG_ABZ_RES2)
	};
    return lo.ab_swap==1;
};
void MT6835_STM32G4::setABSwapped(bool swapped){
    MT6835ABZRes lo = {
			.reg = readRegister(MT6835_REG_ABZ_RES2)
	};
    lo.ab_swap = swapped?1:0;
    writeRegister(MT6835_REG_ABZ_RES2, lo.reg);
};



uint16_t MT6835_STM32G4::getZeroPosition(){
    uint8_t hi = readRegister(MT6835_REG_ZERO1);
    MT6835Options0 lo = {
            .reg = readRegister(MT6835_REG_ZERO2)
    };
    return (hi << 4) | lo.zero_pos_low;
};
void MT6835_STM32G4::setZeroPosition(uint16_t pos){
    uint8_t hi = (pos >> 4);
    MT6835Options0 lo = {
            .reg = readRegister(MT6835_REG_ZERO2)
    };
    lo.zero_pos_low = pos & 0x0F;
    writeRegister(MT6835_REG_ZERO1, hi);
    writeRegister(MT6835_REG_ZERO2, lo.reg);
};



MT6835Options1 MT6835_STM32G4::getOptions1(){
    MT6835Options1 result = {
			.reg = readRegister(MT6835_REG_OPTS1)
	};
    return result;
};
void MT6835_STM32G4::setOptions1(MT6835Options1 opts){
    writeRegister(MT6835_REG_OPTS1, opts.reg);
};



MT6835Options2 MT6835_STM32G4::getOptions2(){
    MT6835Options2 result = {
			.reg = readRegister(MT6835_REG_OPTS2)
	};
    return result;
};
void MT6835_STM32G4::setOptions2(MT6835Options2 opts){
    MT6835Options2 val = getOptions2();
    val.nlc_en = opts.nlc_en;
    val.pwm_fq = opts.pwm_fq;
    val.pwm_pol = opts.pwm_pol;
    val.pwm_sel = opts.pwm_sel;
    writeRegister(MT6835_REG_OPTS2, val.reg);
};



MT6835Options3 MT6835_STM32G4::getOptions3(){
    MT6835Options3 result = {
			.reg = readRegister(MT6835_REG_OPTS3)
	};
    return result;    
};
void MT6835_STM32G4::setOptions3(MT6835Options3 opts){
    MT6835Options3 val = getOptions3();
    val.rot_dir = opts.rot_dir;
    val.hyst = opts.hyst;
    writeRegister(MT6835_REG_OPTS3, val.reg);
};



MT6835Options4 MT6835_STM32G4::getOptions4(){
    MT6835Options4 result = {
			.reg = readRegister(MT6835_REG_OPTS4)
	};
    return result;
};
void MT6835_STM32G4::setOptions4(MT6835Options4 opts){
    MT6835Options4 val = getOptions4();
    val.gpio_ds = opts.gpio_ds;
    val.autocal_freq = opts.autocal_freq;
    writeRegister(MT6835_REG_OPTS4, val.reg);
};


void MT6835_STM32G4::transfer24(MT6835Command* outValue) {
    if (nCS >= 0)
        digitalWrite(nCS, LOW);
    spi->beginTransaction(settings);
    spi->transfer(outValue, 3);
    spi->endTransaction();
    if (nCS >= 0)
        digitalWrite(nCS, HIGH);
};

uint8_t MT6835_STM32G4::readRegister(uint16_t reg) {
    MT6835Command cmd;
    cmd.cmd = MT6835_OP_READ;
    cmd.addr = reg;
    transfer24(&cmd);
    return cmd.data;
};
bool MT6835_STM32G4::writeRegister(uint16_t reg, uint8_t value) {
    MT6835Command cmd;
    cmd.cmd = MT6835_OP_READ;
    cmd.addr = reg;
    cmd.data = value;
    transfer24(&cmd);
    return cmd.data == MT6835_WRITE_ACK;
};