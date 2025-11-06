#include <Pixy2.h>

Pixy2 pixy;


void setup()
{
  Serial.begin(115200);
  Serial.println("Starting Pixy2 Line Tracking (Vector Mode) Example...");

  Serial.println(pixy.init());
  Serial.println(pixy.changeProg("line")); // Switch Pixy2 to line tracking program
}

void loop()
{
  int8_t result = pixy.line.getMainFeatures();

  if (result <= 0) {
    // No line detected
    Serial.println("No line detected");
    delay(100);
    return;
  }

  // If a vector (main line) is detected
  if (pixy.line.numVectors)
  {
    // Read vector data
    int x0 = pixy.line.vectors->m_x0;
    int y0 = pixy.line.vectors->m_y0;
    int x1 = pixy.line.vectors->m_x1;
    int y1 = pixy.line.vectors->m_y1;

    Serial.print("Vector: (");
    Serial.print(x0); Serial.print(", ");
    Serial.print(y0); Serial.print(") -> (");
    Serial.print(x1); Serial.print(", ");
    Serial.print(y1); Serial.println(")");
  }

  // If intersection detected
  if (pixy.line.numIntersections)
  {
    Serial.print("Intersections: ");
    Serial.println(pixy.line.numIntersections);
  }

  delay(200);
}
