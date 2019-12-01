#include <MAX31855.h>

// relay pin
#define RELAY 5    // relay control

// MAX 31855 Pins
#define MAXDO   11
#define MAXCS   10
#define MAXCLK  12

// Initialise the MAX31855 IC for thermocouple tempterature reading
MAX31855 tc(MAXCLK, MAXCS, MAXDO);

bool kilnRelayMode = false;
bool heartbeat = false;
int readSerialValue = 0;
uint32_t lastMessageTime = 0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, LOW);    // turn the LED off by making the voltage LOW

  tc.begin();
  delay(500); // delay for initial temp probe read to be garbage
}

// the loop routine runs over and over again forever:
void loop() {
  int status = tc.read();
  float internal = tc.getInternal();
  float external = tc.getTemperature();

  if(Serial.available() > 0){
    
    Serial.print("start ");
    Serial.print(external);
    Serial.print(" ");
    Serial.print(internal);
    Serial.print(" ");
    Serial.print(kilnRelayMode);
    Serial.print(" ");
    Serial.print(readSerialValue);
    Serial.print(" ");
    Serial.print(heartbeat);
    Serial.println(" end");
    
    readSerialValue = Serial.read();
    while(Serial.available() > 0){
      Serial.read();
    }

    if(readSerialValue == 49){
      kilnRelayMode = true;
      lastMessageTime = millis();
    }
    else{
      if(readSerialValue == 48){
        kilnRelayMode = false;
        lastMessageTime = millis();
      }
      else{
      kilnRelayMode = false;
      }  
    }
    
  }

  if((millis() - lastMessageTime) > 10000){
    kilnRelayMode = false;
    heartbeat = false;
  }
  else{
    heartbeat = true;
  }

  
  digitalWrite(RELAY, kilnRelayMode);   // turn the LED on (HIGH is the voltage level)
  delay(10);
  //kilnRelayMode = !kilnRelayMode; // wait for a second
  //digitalWrite(RELAY, LOW);    // turn the LED off by making the voltage LOW
  //delay(1000);  
  
}
