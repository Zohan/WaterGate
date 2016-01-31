/**
 * A Mirf example to test the latency between two Ardunio.
 *
 * Pins:
 * Hardware SPI:
 * MISO -> 12
 * MOSI -> 11
 * SCK -> 13
 *
 * Configurable:
 * CE -> 8
 * CSN -> 7
 *
 * Note: To see best case latency comment out all Serial.println
 * statements not displaying the result and load 
 * 'ping_server_interupt' on the server.
 */
#define LOG_OUT 1 // use the log output function
#define FFT_N 256 // set to 256 point fft

#include <FFT.h> // include the library
#include "cc2500_REG.h"
#include "cc2500_VAL.h"
#include <SPI.h>
#include <TimerOne.h>
byte ADC_CHANNEL = 0;

volatile uint16_t samplePos = 0;
volatile uint16_t adcCounter = 0;
const byte adcSampleRateOffset = 12;

#define CC2500_IDLE    0x36      // Exit RX / TX, turn
#define CC2500_TX      0x35      // Enable TX. If in RX state, only enable TX if CCA passes
#define CC2500_RX      0x34      // Enable RX. Perform calibration if enabled
#define CC2500_FTX     0x3B      // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC2500_FRX     0x3A      // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC2500_TXFIFO  0x3F
#define CC2500_RXFIFO  0x3F
#define No_of_Bytes    6
#define TX_TIMEOUT 10000 // in milliseconds

long previousTXTimeoutMillis = 0;
long sampleInterval = 33; // Microseconds
long previousMillis, previousMicros;
long sendInterval = 200; // in milliseconds
uint16_t moistureSensor1, moistureSensor2;
byte fftData;

void setup(){
  Serial.begin(57600);
  
  // Setup 
  pinMode(SS,OUTPUT);
  SPI.begin();
  digitalWrite(SS,HIGH);

  Serial.println("Initializing Wireless..");
  init_CC2500();
  Read_Config_Regs();
  Serial.println("Beginning ... ");  
  Timer1.initialize(1000);
  Timer1.attachInterrupt(sampleAudio);
}

void loop(){
  unsigned long currentMillis = millis();
  unsigned long currentMicros = micros();
  getMoistureSensorValues();
  if(samplePos>=FFT_N) analyzeSound();
  if(currentMillis - previousMillis > sendInterval) {
    previousMillis = currentMillis;
    Timer1.stop();
    sendPacket(4, fftData);

    //delay(10);
    
    if(moistureSensor1 > 600) sendPacket(5, 0);
    else if (moistureSensor1 > 400) sendPacket(5, 1);
    else sendPacket(5, 2);
    //delay(10);
    if(moistureSensor2 > 600) sendPacket(6, 0);
    else if (moistureSensor2 > 400) sendPacket(6, 1);
    else sendPacket(6, 2);
    Timer1.initialize(1000);
    Timer1.attachInterrupt(sampleAudio);
  }
  //Serial.println(analogRead(A0));
}

void sampleAudio() {
  cli();
  fft_input[samplePos] = analogRead(A0); // put real data into bins
  samplePos++;
  //Serial.println(analogRead(A0));
  sei();
}

void analyzeSound() {
  cli();  // UDRE interrupt slows this way down on arduino1.0
  samplePos = 0;
  fft_window(); // window the data for better frequency response
  fft_reorder(); // reorder the data before doing the fht
  fft_run(); // process the data in the fht
  fft_mag_log(); // take the output of the fht
  //Serial.print("Time: ");
  //Serial.println(micros());
  uint8_t biggestMagnitude = 0;
  int biggestBin = 0;
  for (uint8_t i = 0 ; i < FFT_N/2 ; i++) {
    if(fft_log_out[i] >= 100 && i >= 8) {
      if(fft_log_out[i] > biggestMagnitude) {
        biggestMagnitude = fft_log_out[i];
        biggestBin = i;
      }
      Serial.print("Bin: ");
      Serial.print(i); // send out the data
      Serial.print(" Freq: ");
      Serial.print(fft_log_out[i]); // send out the data
      Serial.println(" ");
      if(i==20) {
        fftData = 1;
      }
    }
    //Serial.print(fft_log_out[i]); // send out the data
    //Serial.print(" ");
  }
  //Serial.println();
  /*Serial.print("Biggest Bin: ");
  Serial.print(biggestBin);
  Serial.print(" ");
  Serial.println(biggestMagnitude);*/
  sei();
}

void getMoistureSensorValues() {
  moistureSensor1 = analogRead(A2);
  moistureSensor2 = analogRead(A3);
  //Serial.print(moistureSensor1);
  //Serial.print(" ");
  //Serial.println(moistureSensor2);
}

void sendPacket(byte sensorId, byte sensorValue) {
  WriteReg(REG_IOCFG1,0x06);
  // Make sure that the radio is in IDLE state before flushing the FIFO
  SendStrobe(CC2500_IDLE);
  // Flush TX FIFO
  SendStrobe(CC2500_FTX);
  // prepare Packet
  int length = 3;
  unsigned char packet[length];
  // First Byte = Length Of Packet
  packet[0] = length;
  packet[1] = sensorId;
  packet[2] = sensorValue;
  
  // SIDLE: exit RX/TX
  SendStrobe(CC2500_IDLE);
  
  Serial.println("Transmitting ");
  for(int i = 0; i < length; i++)
  {	  
      WriteReg(CC2500_TXFIFO,packet[i]);
  }
  // STX: enable TX
  SendStrobe(CC2500_TX);
  // Wait for GDO0 to be set -> sync transmitted
  previousTXTimeoutMillis = millis();
  while (!digitalRead(MISO) && (millis() - previousTXTimeoutMillis) <= TX_TIMEOUT) {
     //Serial.println("GDO0 = 0");
  }
   
  // Wait for GDO0 to be cleared -> end of packet
  previousTXTimeoutMillis = millis();
  while (digitalRead(MISO) && (millis() - previousTXTimeoutMillis) <= TX_TIMEOUT) {
      //Serial.println("GDO0 = 1");
  }
  //Serial.println("Finished sending");
  SendStrobe(CC2500_IDLE);
}

void WriteReg(char addr, char value){
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) {
    };
  SPI.transfer(addr);
  SPI.transfer(value);
  digitalWrite(SS,HIGH);
}

char ReadReg(char addr){
  addr = addr + 0x80;
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) {
    };
  char x = SPI.transfer(addr);
  char y = SPI.transfer(0);
  digitalWrite(SS,HIGH);
  return y;  
}

char SendStrobe(char strobe){
  digitalWrite(SS,LOW);
  
  while (digitalRead(MISO) == HIGH) {
  };
    
  char result =  SPI.transfer(strobe);
  digitalWrite(SS,HIGH);
  return result;
}

void init_CC2500(){
  WriteReg(REG_IOCFG2,0x06);
  WriteReg(REG_IOCFG1,0x06);
  WriteReg(REG_IOCFG0,0x01);
  WriteReg(REG_FIFOTHR, SMARTRF_SETTING_FIFOTHR);
  WriteReg(REG_SYNC1,SMARTRF_SETTING_SYNC1);
  WriteReg(REG_SYNC0,SMARTRF_SETTING_SYNC0);
  WriteReg(REG_PKTLEN,SMARTRF_SETTING_PKTLEN);
  WriteReg(REG_PKTCTRL1,SMARTRF_SETTING_PKTCTRL1);
  WriteReg(REG_PKTCTRL0, SMARTRF_SETTING_PKTCTRL0);
  
  WriteReg(REG_ADDR,SMARTRF_SETTING_ADDR);
  WriteReg(REG_CHANNR,SMARTRF_SETTING_CHANNR);
  WriteReg(REG_FSCTRL1,SMARTRF_SETTING_FSCTRL1);
  WriteReg(REG_FSCTRL0,SMARTRF_SETTING_FSCTRL0);
  WriteReg(REG_FREQ2,SMARTRF_SETTING_FREQ2);
  WriteReg(REG_FREQ1,SMARTRF_SETTING_FREQ1);
  WriteReg(REG_FREQ0,SMARTRF_SETTING_FREQ0);
  WriteReg(REG_MDMCFG4,SMARTRF_SETTING_MDMCFG4);
  WriteReg(REG_MDMCFG3,SMARTRF_SETTING_MDMCFG3);
  WriteReg(REG_MDMCFG2,SMARTRF_SETTING_MDMCFG2);
  WriteReg(REG_MDMCFG1,SMARTRF_SETTING_MDMCFG1);
  WriteReg(REG_MDMCFG0,SMARTRF_SETTING_MDMCFG0);
  WriteReg(REG_DEVIATN,SMARTRF_SETTING_DEVIATN);
  WriteReg(REG_MCSM2,SMARTRF_SETTING_MCSM2);
  WriteReg(REG_MCSM1,SMARTRF_SETTING_MCSM1);
  WriteReg(REG_MCSM0,SMARTRF_SETTING_MCSM0);
  WriteReg(REG_FOCCFG,SMARTRF_SETTING_FOCCFG);

  WriteReg(REG_BSCFG,SMARTRF_SETTING_BSCFG);
  WriteReg(REG_AGCCTRL2,SMARTRF_SETTING_AGCCTRL2);
  WriteReg(REG_AGCCTRL1,SMARTRF_SETTING_AGCCTRL1);
  WriteReg(REG_AGCCTRL0,SMARTRF_SETTING_AGCCTRL0);
  WriteReg(REG_WOREVT1,SMARTRF_SETTING_WOREVT1);
  WriteReg(REG_WOREVT0,SMARTRF_SETTING_WOREVT0);
  WriteReg(REG_WORCTRL,SMARTRF_SETTING_WORCTRL);
  WriteReg(REG_FREND1,SMARTRF_SETTING_FREND1);
  WriteReg(REG_FREND0,SMARTRF_SETTING_FREND0);
  WriteReg(REG_FSCAL3,SMARTRF_SETTING_FSCAL3);
  WriteReg(REG_FSCAL2,SMARTRF_SETTING_FSCAL2);
  WriteReg(REG_FSCAL1,SMARTRF_SETTING_FSCAL1);
  WriteReg(REG_FSCAL0,SMARTRF_SETTING_FSCAL0);
  WriteReg(REG_RCCTRL1,SMARTRF_SETTING_RCCTRL1);
  WriteReg(REG_RCCTRL0,SMARTRF_SETTING_RCCTRL0);
  WriteReg(REG_FSTEST,SMARTRF_SETTING_FSTEST);
  WriteReg(REG_PTEST,SMARTRF_SETTING_PTEST);
  WriteReg(REG_AGCTEST,SMARTRF_SETTING_AGCTEST);
  WriteReg(REG_TEST2,SMARTRF_SETTING_TEST2);
  WriteReg(REG_TEST1,SMARTRF_SETTING_TEST1);
  WriteReg(REG_TEST0,SMARTRF_SETTING_TEST0);
}

void Read_Config_Regs(void){ 
  Serial.println(ReadReg(REG_IOCFG2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_IOCFG1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_IOCFG0),HEX);
   delay(10);
/* Serial.println(ReadReg(REG_FIFOTHR),HEX);
   delay(10);
  Serial.println(ReadReg(REG_SYNC1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_SYNC0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_PKTLEN),HEX);
   delay(10);
  Serial.println(ReadReg(REG_PKTCTRL1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_PKTCTRL0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_ADDR),HEX);
   delay(10);
  Serial.println(ReadReg(REG_CHANNR),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCTRL1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCTRL0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FREQ2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FREQ1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FREQ0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MDMCFG4),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MDMCFG3),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MDMCFG2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MDMCFG1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MDMCFG0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_DEVIATN),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MCSM2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MCSM1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MCSM0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FOCCFG),HEX);
   delay(10);

  Serial.println(ReadReg(REG_BSCFG),HEX);
   delay(10);
  Serial.println(ReadReg(REG_AGCCTRL2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_AGCCTRL1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_AGCCTRL0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_WOREVT1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_WOREVT0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_WORCTRL),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FREND1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FREND0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCAL3),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCAL2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCAL1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCAL0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_RCCTRL1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_RCCTRL0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSTEST),HEX);
   delay(10);
  Serial.println(ReadReg(REG_PTEST),HEX);
   delay(10);
  Serial.println(ReadReg(REG_AGCTEST),HEX);
   delay(10);
  Serial.println(ReadReg(REG_TEST2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_TEST1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_TEST0),HEX);
   delay(10);
 /*
  Serial.println(ReadReg(REG_PARTNUM),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_VERSION),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_FREQEST),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_LQI),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_RSSI),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_MARCSTATE),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_WORTIME1),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_WORTIME0),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_PKTSTATUS),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_VCO_VC_DAC),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_TXBYTES),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_RXBYTES),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_RCCTRL1_STATUS),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_RCCTRL0_STATUS),HEX);
   delay(1000);
*/  
}
  



