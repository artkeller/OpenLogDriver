// Example_BasicLogging.ino
// Zeigt die grundlegende Initialisierung und das Schreiben von Daten in den OpenLog.

#include "OpenLogDriver.h"

#define OPENLOG_TX_PIN 21
#define OPENLOG_RX_PIN 20
#define OPENLOG_DTR_GRN_PIN 10

OpenLogDriver openLog(Serial1, OPENLOG_RX_PIN, OPENLOG_TX_PIN, OPENLOG_DTR_GRN_PIN);

void setup() {
  Serial.begin(115200);
  delay(2000);
  while (!Serial)
    ;
  Serial.println("OpenLog Debug Test");
}

void loop() {
  static bool initialized = false;

  if (!initialized) {
    OpenLogDriver::InitState state = openLog.beginAsync();

    switch (state) {
      case OpenLogDriver::InitState::COMPLETE:
        Serial.println("Initialization successful!");
        initialized = true;
        break;

      case OpenLogDriver::InitState::ERROR:
        Serial.println("Critical error!");
        while (1) {
          delay(1000);
          Serial.println("Halted due to error");
        }
        break;

      default:
        delay(10);
        break;
    }
  } else {
    // Normales Logging
    static unsigned long seqId = 0;
    static unsigned long lastLogTime = 0;
    const unsigned long logInterval = 2000;

    if (millis() - lastLogTime >= logInterval) {
      lastLogTime = millis();
      seqId++;

      String logData = "SeqID: " + String(seqId) + ", Timestamp: " + String(millis()) + ", Value: " + String(random(0, 100));
      Serial.print("Schreibe Daten: ");
      Serial.println(logData);

      OpenLogStatus status = openLog.write(logData + "\r\n");
      if (status != OpenLogStatus::OK) {
        Serial.print("Fehler beim Schreiben: ");
        Serial.println(static_cast<int>(status));
      }
    }
  }

  // Kurze Pause für Stabilität
  delay(10);
}
