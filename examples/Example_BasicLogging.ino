// Example_BasicLogging.ino
// Zeigt die grundlegende Initialisierung und das Schreiben von Daten in den OpenLog.

#include "OpenLogDriver.h"

// Definiere die Pins für die serielle Kommunikation und den Reset
#define OPENLOG_RX_PIN 21 // ESP32 GPIO21 (RX1) -> OpenLog TXO
#define OPENLOG_TX_PIN 20 // ESP32 GPIO20 (TX1) -> OpenLog RXI
#define OPENLOG_DTR_GRN_PIN 10 // ESP33 GPIO10 -> OpenLog GRN (Reset)

// Erstelle eine Instanz des OpenLogTreiber
// Verwende Serial1 für die Kommunikation mit dem OpenLog
OpenLogDriver openLog(Serial1, OPENLOG_DTR_GRN_PIN);

void setup() {
    Serial.begin(115200); // Serielle Kommunikation für Debug-Ausgaben
    while (!Serial) {
        ; // Warte auf Serial Monitor
    }
    Serial.println("Starte Basic Logging Beispiel...");

    // Initialisiere den OpenLog-Treiber
    // Der begin() Aufruf führt Hardware-Reset, Baudraten-Sondierung und Firmware-Erkennung durch.
    OpenLogFirmwareType detectedFirmware = openLog.begin(10000); // 10 Sekunden Timeout

    if (detectedFirmware == OpenLogFirmwareType::UNKNOWN) {
        Serial.println("FEHLER: OpenLog konnte nicht initialisiert werden!");
        while (true) {
            delay(100);
        } // Endlosschleife bei Fehler
    }

    Serial.println("OpenLog ist bereit zum Loggen.");
    Serial.print("Erkannte Firmware: ");
    switch (detectedFirmware) {
        case OpenLogFirmwareType::OPENLOG_STANDARD: Serial.println("OpenLog Standard"); break;
        case OpenLogFirmwareType::OPENLOG_LIGHT:    Serial.println("OpenLog Light");    break;
        case OpenLogFirmwareType::OPENLOG_MINIMAL:  Serial.println("OpenLog Minimal");  break;
        default: Serial.println("Unbekannt"); break;
    }
    Serial.print("Aktuelle Baudrate: ");
    Serial.println(openLog.getCurrentBaudRate());
}

void loop() {
    static unsigned long lastLogTime = 0;
    const unsigned long logInterval = 2000; // Alle 2 Sekunden loggen

    if (millis() - lastLogTime >= logInterval) {
        lastLogTime = millis();

        String logData = "Timestamp: " + String(millis()) + ", Value: " + String(random(0, 100));
        Serial.print("Schreibe Daten: ");
        Serial.println(logData);

        OpenLogStatus status = openLog.write(logData + "\r\n"); // Zeilenumbruch für Lesbarkeit
        if (status!= OpenLogStatus::OK) {
            Serial.print("FEHLER beim Schreiben in OpenLog: ");
            if (status == OpenLogStatus::NOT_SUPPORTED) {
                Serial.println("Funktion nicht unterstützt (Firmware-Typ).");
            } else if (status == OpenLogStatus::TIMEOUT) {
                Serial.println("Timeout.");
            } else {
                Serial.println("Allgemeiner Fehler.");
            }
        } else {
            Serial.println("Daten erfolgreich geschrieben.");
        }

        // Optional: Daten synchronisieren, um sicherzustellen, dass sie auf die SD-Karte geschrieben werden
        // Dies kann die Performance beeinträchtigen, erhöht aber die Datenintegrität.
        // if (openLog.syncData() == OpenLogStatus::OK) {
        //     Serial.println("Daten synchronisiert.");
        // } else {
        //     Serial.println("FEHLER beim Synchronisieren der Daten.");
        // }
    }
}