#include <Arduino.h>
#include "Wire.h"
#include <ESP32SPISlave.h>


#define LED1 12
#define LED2 14
#define LED3 27
#define POT 34
#define I2C_DEV_ADDR 0x55  

ESP32SPISlave slave; 

static constexpr uint32_t BUFFER_SIZE {32};
uint8_t spi_slave_tx_buf [BUFFER_SIZE];
uint8_t spi_slave_rx_buf [BUFFER_SIZE];
char cmd;


void recibirSPI();
void enviarI2C();
void RecibirI2C(int len);

void setup(){
  Serial.begin(115200);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(POT, INPUT);

  slave.setDataMode(SPI_MODE0);
  slave.setQueueSize(1);
  slave.begin(VSPI);
  memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
  memset(spi_slave_rx_buf, 0, BUFFER_SIZE);

  Wire.onReceive(RecibirI2C);
  Wire.onRequest(enviarI2C);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
}

void loop(){
recibirSPI();
  
}

void enviarI2C(){
  int poten = analogRead(POT);
  int lecPot = map(poten, 0, 4095, 0, 255);
  Wire.write(lecPot);
  Serial.println("onrequest");
}

void RecibirI2C(int len){
  Serial.printf("onReceive[%d]", len);
  while(Wire.available()){
    uint8_t incomingByte = Wire.read();
    Serial.write(incomingByte);
  if (incomingByte == 'S'){
    enviarI2C();
  }
  else {
    Serial.println("no es posible enviar dato");
  }
  }
}

void recibirSPI(){
  slave.wait(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);
  while (slave.available())
  {
    cmd = spi_slave_rx_buf[0];
    slave.pop();
  }
  if (cmd == '1'){
    digitalWrite(LED1, HIGH);
    delay(2000);
    digitalWrite(LED1, LOW);
    Serial.println("encendiendo led 1");
  }
  else if(cmd == '2'){
    digitalWrite(LED2, HIGH);
    delay(2000);
    digitalWrite(LED2, LOW);
    Serial.println("encendiendo led 2");
  }
  else if (cmd == '3'){
    digitalWrite(LED3,HIGH);
    delay(2000);
    digitalWrite(LED3, LOW);
    Serial.println("encendiendo led 3");
  }
  
}