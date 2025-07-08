// Example_VoltageMonitoring.ino
// Zeigt die Spannungsüberwachung und Reaktion auf niedrige Batteriespannung.

#include "OpenLogDriver.h"

// Definiere die Pins für die serielle Kommunikation und den Reset
#define OPENLOG_RX_PIN 21 // ESP32 GPIO21 (RX1) -> OpenLog TXO
#define OPENLOG_TX_PIN 20 // ESP32 GPIO20 (TX1) -> OpenLog RXI
#define OPENLOG_DTR_GRN_PIN 10 // ESP32 GPIO10 -> OpenLog GRN (Reset)

// Pins für die Spannungsüberwachung
#define VOLTAGE_ADC_PIN 36 // Beispiel-ADC-Pin am ESP32 (GPIO36 ist ADC1_CHANNEL0)
// WICHTIG: Dies ist ein Beispielwert. Passen Sie ihn an Ihr Widerstandsteiler-Netzwerk an.
// Wenn Sie z.B. eine 5V-Batterie mit einem 10k/20k-Teiler auf 3.3V skalieren, wäre das Verhältnis 3.3 / 5 = 0.66
// Wenn Sie direkt die 3.3V VCC des ESP32 messen, wäre das Verhältnis 1.0
#define VOLTAGE_DIVIDER_RATIO 1.0 // Beispiel: Direkte Messung der 3.3V VCC
#define BROWNOUT_THRESHOLD 2.8   // Beispiel: Spannung unter 2.8V als kritisch ansehen

// Erstelle eine Instanz des OpenLogTreiber
OpenLogDriver openLog(Serial1, OPENLOG_DTR_GRN_PIN);

void setup() {
    Serial.begin(115200); // Serielle Kommunikation für Debug-Ausgaben
    while (!Serial) {
        ; // Warte auf Serial Monitor
    }
    Serial.println("Starte Spannungsüberwachungs Beispiel...");

    // Konfiguriere die Spannungsüberwachung
    openLog.setVoltageMonitoring(VOLTAGE_ADC_PIN, VOLTAGE_DIVIDER_RATIO, BROWNOUT_THRESHOLD);

    // Initialisiere den OpenLog-Treiber
    OpenLogFirmwareType detectedFirmware = openLog.begin(10000);

    if (detectedFirmware == OpenLogFirmwareType::UNKNOWN) {
        Serial.println("FEHLER: OpenLog konnte nicht initialisiert werden!");
        while (true) {
            delay(100);
        }
    }
    Serial.println("OpenLog ist bereit zum Loggen.");
}

void loop() {
    static unsigned long lastCheckTime = 0;
    const unsigned long checkInterval = 5000; // Alle 5 Sekunden Spannung prüfen

    if (millis() - lastCheckTime >= checkInterval) {
        lastCheckTime = millis();

        OpenLogStatus powerStatus = openLog.checkPowerStatus();
        String logMessage = "Timestamp: " + String(millis()) + ", ";

        if (powerStatus == OpenLogStatus::OK) {
            logMessage += "Power: OK";
            Serial.println("Power Status: OK");
        } else if (powerStatus == OpenLogStatus::LOW_VOLTAGE_WARNING) {
            logMessage += "Power: LOW_VOLTAGE_WARNING";
            Serial.println("Power Status: NIEDRIGE SPANNUNG WARNUNG!");
            // Hier könnte der Sketch nicht-essentielle Funktionen deaktivieren
        } else if (powerStatus == OpenLogStatus::BROWNOUT_IMMINENT) {
            logMessage += "Power: BROWNOUT_IMMINENT";
            Serial.println("Power Status: BROWNOUT DROHT! Leite Notfall-Shutdown ein.");

            // KRITISCHE REAKTION: Daten synchronisieren und Logging unterbrechen
            openLog.write("CRITICAL: " + logMessage + "\r\n"); // Letzte Warnung loggen
            openLog.syncData(); // Sicherstellen, dass alle Daten geschrieben werden

            Serial.println("Logging unterbrochen. System wird heruntergefahren.");
            // Hier könnte der Sketch das System in den Tiefschlaf versetzen oder ausschalten
            while (true) {
                delay(100);
            } // Endlosschleife nach kritischem Fehler
        } else {
            logMessage += "Power: UNKNOWN_ERROR";
            Serial.println("Power Status: UNBEKANNTER FEHLER bei Spannungsüberwachung.");
        }

        // Logge den Power-Status in den OpenLog
        OpenLogStatus writeStatus = openLog.write(logMessage + "\r\n");
        if (writeStatus!= OpenLogStatus::OK) {
            Serial.println("FEHLER beim Schreiben des Power-Status in OpenLog.");
        }
    }

    // Normale Logging-Operationen fortsetzen, solange der Power-Status OK ist
    if (openLog.getOpenLogMode() == OpenLogMode::LOGGING_MODE && openLog.checkPowerStatus() == OpenLogStatus::OK) {
        // Beispiel für kontinuierliches Logging
        // openLog.write("Normal data point.\r\n");
        // delay(100);
    }
}
