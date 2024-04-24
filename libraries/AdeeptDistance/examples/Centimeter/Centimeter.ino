#include <Adeept_Distance.h>

Adeept_Distance Dist;
int distance;

void setup()
{
  Serial.begin(9600);
  Dist.begin(5,3);
}

void loop()
{
  distance = Dist.getDistanceCentimeter();
  Serial.print("\nDistance in centimers: ");
  Serial.print(distance);  
  delay(500); //make it readable
}
