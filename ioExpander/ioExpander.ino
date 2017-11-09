#include <Wire.h>
#include <Adafruit_MCP23017.h>

Adafruit_MCP23017 mcp;

void setup() {
  // put your setup code here, to run once:
  mcp.begin();

  mcp.pinMode(0,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  mcp.digitalWrite(0, HIGH);
}
