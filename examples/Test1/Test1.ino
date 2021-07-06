#include <JackVan.h>

JackVan Van1(27,26,34,33,32,35,19,23,4,18);

void setup()
{
}

void loop()
{

  Van1.read_Vbat();

  /*
  Van1.dot(); Van1.dot(); Van1.dot();
  Van1.dash(); Van1.dash(); Van1.dash();
  Van1.dot(); Van1.dot(); Van1.dot();
  */

  delay(3000);

}