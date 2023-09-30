
#pragma once

#define PIN_NEOPIX      PA10
#define PIN_FAN         PB7
#define PIN_CANSTDBY    PB6
#define PIN_FDCAN_RX    PB8
#define PIN_FDCAN_TX    PB9

#define PIN_SPI3_MOSI   PB5
#define PIN_SPI3_MISO   PB4
#define PIN_SPI3_SCK    PB3
#define PIN_SPI3_SS     PA15

#define PIN_SPI_SS      PD2




// I2C2
#define INTERNAL_I2C2_SDA PA8
#define INTERNAL_I2C2_SCL PA9


//4th FET driver 
#define PHASE_DH        PA14 // Pull-Down
#define PHASE_DL        PB0 // Pull-Down




//Unused PINS
#define UNUSE_1     PF1 
#define UNUSE_2     PC3 
#define UNUSE_3     PC2
#define UNUSE_4     PC1 
#define UNUSE_5     PC0
#define UNUSE_6     PF10
#define UNUSE_7     PF9
#define UNUSE_8     PC15
#define UNUSE_9     PC14
#define UNUSE_10    PC13
#define UNUSE_11    PE6
#define UNUSE_12    PE5
#define UNUSE_13    PE4
#define UNUSE_14    PE3
#define UNUSE_15    PE2
#define UNUSE_16    PE1
#define UNUSE_17    PD7
#define UNUSE_18    PD6
#define UNUSE_19    PD5
#define UNUSE_20    PD4
#define UNUSE_21    PD3
#define UNUSE_22    PD1
#define UNUSE_23    PD0
#define UNUSE_24    PC12
#define UNUSE_25    PC11
#define UNUSE_26    PC10
#define UNUSE_27    PC7
#define UNUSE_28    PC6
#define UNUSE_29    PD15
#define UNUSE_30    PD14
#define UNUSE_31    PD13
#define UNUSE_32    PD12
#define UNUSE_33    PD11
#define UNUSE_34    PD10
#define UNUSE_35    PD9
#define UNUSE_36    PD8
#define UNUSE_37    PB15
#define UNUSE_38    PB14
#define UNUSE_39    PB13
#define UNUSE_40    PB12
#define UNUSE_41    PB11
#define UNUSE_42    PB10
#define UNUSE_43    PB2
#define UNUSE_44    PB1


void Pins_State(){


Serial.println("Setting Unused Pins!");

 pinMode(UNUSE_1, OUTPUT);
 digitalWrite(UNUSE_1, LOW);
 pinMode(UNUSE_2, OUTPUT);
 digitalWrite(UNUSE_2, LOW);
 pinMode(UNUSE_3, OUTPUT);
 digitalWrite(UNUSE_3, LOW);
 pinMode(UNUSE_4, OUTPUT);
 digitalWrite(UNUSE_4, LOW);
 pinMode(UNUSE_5, OUTPUT);
 digitalWrite(UNUSE_5, LOW);
 pinMode(UNUSE_6, OUTPUT);
 digitalWrite(UNUSE_6, LOW);
 pinMode(UNUSE_7, OUTPUT);
 digitalWrite(UNUSE_7, LOW);
 pinMode(UNUSE_8, OUTPUT);
 digitalWrite(UNUSE_8, LOW);
 pinMode(UNUSE_9, OUTPUT);
 digitalWrite(UNUSE_9, LOW);
 pinMode(UNUSE_10, OUTPUT);
 digitalWrite(UNUSE_10, LOW);
 pinMode(UNUSE_11, OUTPUT);
 digitalWrite(UNUSE_11, LOW);
 pinMode(UNUSE_12, OUTPUT);
 digitalWrite(UNUSE_12, LOW);
 pinMode(UNUSE_13, OUTPUT);
 digitalWrite(UNUSE_13, LOW);
 pinMode(UNUSE_14, OUTPUT);
 digitalWrite(UNUSE_14, LOW);
 pinMode(UNUSE_15, OUTPUT);
 digitalWrite(UNUSE_15, LOW);
 pinMode(UNUSE_16, OUTPUT);
 digitalWrite(UNUSE_16, LOW);
 pinMode(UNUSE_17, OUTPUT);
 digitalWrite(UNUSE_17, LOW);
 pinMode(UNUSE_18, OUTPUT);
 digitalWrite(UNUSE_18, LOW);
 pinMode(UNUSE_19, OUTPUT);
 digitalWrite(UNUSE_19, LOW);
 pinMode(UNUSE_20, OUTPUT);
 digitalWrite(UNUSE_20, LOW);
 pinMode(UNUSE_21, OUTPUT);
 digitalWrite(UNUSE_21, LOW);
 pinMode(UNUSE_22, OUTPUT);
 digitalWrite(UNUSE_22, LOW);
 pinMode(UNUSE_23, OUTPUT);
 digitalWrite(UNUSE_23, LOW);
 pinMode(UNUSE_24, OUTPUT);
 digitalWrite(UNUSE_24, LOW);
 pinMode(UNUSE_25, OUTPUT);
 digitalWrite(UNUSE_25, LOW);
 pinMode(UNUSE_26, OUTPUT);
 digitalWrite(UNUSE_26, LOW);
 pinMode(UNUSE_27, OUTPUT);
 digitalWrite(UNUSE_27, LOW);
 pinMode(UNUSE_28, OUTPUT);
 digitalWrite(UNUSE_28, LOW);
 pinMode(UNUSE_29, OUTPUT);
 digitalWrite(UNUSE_29, LOW);
 pinMode(UNUSE_30, OUTPUT);
 digitalWrite(UNUSE_30, LOW);
 pinMode(UNUSE_31, OUTPUT);
 digitalWrite(UNUSE_31, LOW);
 pinMode(UNUSE_32, OUTPUT);
 digitalWrite(UNUSE_32, LOW);
 pinMode(UNUSE_33, OUTPUT);
 digitalWrite(UNUSE_33, LOW);
 pinMode(UNUSE_34, OUTPUT);
 digitalWrite(UNUSE_34, LOW);
 pinMode(UNUSE_35, OUTPUT);
 digitalWrite(UNUSE_35, LOW);
 pinMode(UNUSE_36, OUTPUT);
 digitalWrite(UNUSE_36, LOW);
 pinMode(UNUSE_37, OUTPUT);
 digitalWrite(UNUSE_37, LOW);
 pinMode(UNUSE_38, OUTPUT);
 digitalWrite(UNUSE_38, LOW);
 pinMode(UNUSE_39, OUTPUT);
 digitalWrite(UNUSE_39, LOW);
 pinMode(UNUSE_40, OUTPUT);
 digitalWrite(UNUSE_40, LOW);
 pinMode(UNUSE_41, OUTPUT);
 digitalWrite(UNUSE_41, LOW);
 pinMode(UNUSE_42, OUTPUT);
 digitalWrite(UNUSE_42, LOW);
 pinMode(UNUSE_43, OUTPUT);
 digitalWrite(UNUSE_43, LOW);
 pinMode(UNUSE_44, OUTPUT);
 digitalWrite(UNUSE_44, LOW);



//Internal driver pins - These are pulled-down. We still set them as outputs and pull them to GND.
pinMode(PHASE_AH, OUTPUT); // Internal Pull-Down
digitalWrite(PHASE_AH, LOW); // Internal Pull-Down

pinMode(PHASE_AL, OUTPUT); // Internal Pull-Down
digitalWrite(PHASE_AL, LOW); // Internal Pull-Down

pinMode(PHASE_BH, OUTPUT); // Internal Pull-Down
digitalWrite(PHASE_BH, LOW); // Internal Pull-Down

pinMode(PHASE_BL, OUTPUT); // Internal Pull-Down
digitalWrite(PHASE_BL, LOW); // Internal Pull-Down

pinMode(PHASE_CH, OUTPUT); // Internal Pull-Down
digitalWrite(PHASE_CH, LOW); // Internal Pull-Down

pinMode(PHASE_CL, OUTPUT); // Internal Pull-Down
digitalWrite(PHASE_CL, LOW); // Internal Pull-Down


//4th FET driver 
pinMode(PHASE_DH, OUTPUT); // External Pull-Down
digitalWrite(PHASE_DH, LOW); // External Pull-Down

pinMode(PHASE_DL, OUTPUT);
digitalWrite(PHASE_DL, LOW);


//Internal signaling pins

pinMode(INTERNAL_READY, INPUT);
digitalWrite(INTERNAL_READY, HIGH);

pinMode(INTERNAL_nFAULT, INPUT);
digitalWrite(INTERNAL_nFAULT, HIGH);


// CAN FD pins - These are pulled HIGH

pinMode(PIN_FDCAN_RX, INPUT);
digitalWrite(PIN_FDCAN_RX, HIGH);

pinMode(PIN_CANSTDBY, OUTPUT);
digitalWrite(PIN_CANSTDBY, HIGH);

pinMode(PIN_FDCAN_TX, OUTPUT);
digitalWrite(PIN_FDCAN_TX, HIGH);

// Fan PWM Pin
pinMode(PIN_FAN, OUTPUT);
digitalWrite(PIN_FAN, LOW);

//SPI3 Pins
pinMode(PIN_SPI3_MOSI, INPUT);
digitalWrite(PIN_SPI3_MOSI, HIGH);

pinMode(PIN_SPI3_MISO, INPUT);
digitalWrite(PIN_SPI3_MISO, HIGH);

pinMode(PIN_SPI3_SCK, INPUT);
digitalWrite(PIN_SPI3_SCK, HIGH);


pinMode(PIN_SPI3_SS, OUTPUT);
digitalWrite(PIN_SPI3_SS, LOW);

pinMode(PIN_SPI_SS, OUTPUT);
digitalWrite(PIN_SPI_SS, HIGH);


return;

}



/*

//SPI 
uint8_t data0 = 0x00;
uint8_t data1 = 0x00;
uint8_t angle_buffer[3];
uint8_t data5 = 0x00;

uint32_t getRaw21bitMT6835(){


  digitalWrite(PD2, LOW); // manually take CSN low for SPI_1 transmission
  data0 = SPI.transfer(0xA0, SPI_CONTINUE); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  data1 = SPI.transfer(0x003, SPI_CONTINUE); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  angle_buffer[0] = SPI.transfer(0x00, SPI_CONTINUE); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  angle_buffer[1] = SPI.transfer(0x00, SPI_CONTINUE); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  angle_buffer[2] = SPI.transfer(0x00, SPI_CONTINUE); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  data5 = SPI.transfer(0x00, SPI_LAST); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  digitalWrite(PD2, HIGH); // manually take CSN high between spi transmissions

  
  SerialUSB.println("Read Register 0x003:  ");
  SerialUSB.println(data0, BIN);
  SerialUSB.println(data1, BIN);
  SerialUSB.println(angle_buffer[0], BIN);
  SerialUSB.println(angle_buffer[1], BIN);
  SerialUSB.println(angle_buffer[2], BIN);
  SerialUSB.println(data5, BIN);
  

  uint32_t buff_shift = (angle_buffer[0] << 13) | (angle_buffer[1] << 5) | (angle_buffer[2] >> 3);
  return buff_shift;

  }

  */





