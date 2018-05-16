#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"


RF24 radio(5, A1);  //pin used for CE/CSN

const uint64_t addr = 0x1111111111;

const int taille = 32;

char message[taille + 1]; 

void setup(void)
{

  Serial.begin(115200);

  Serial.println("Data receiver");

  radio.begin();

  radio.openReadingPipe(1, addr);

  radio.startListening();
}

void loop(void)
{
  while ( radio.available() )
  {
    radio.read( message, taille );

    Serial.print("The message is: ");

    Serial.println(message);   //It is esay to see if a message is missing : in this case, the line with the corresponding number doesn't appear on the monitor
  }
}
