#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

int comp = 0;

RF24 radio(5, A1); //pin used for CE/CSN

const uint64_t addr = 0x1111111111;

const int taille = 32;

char message[taille + 1];



void setup(void)
{
  Serial.begin(115200);

  Serial.println("Data transmitter");

  radio.begin();

  radio.openWritingPipe(addr);

}

void loop(void)
{

  compt++;

  itoa(compt, message, 10);

  Serial.print("Sent message ");
  Serial.println(message);

  radio.write( message, taille );

  delay(1000);

}
