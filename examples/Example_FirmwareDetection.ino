// Example_FirmwareDetection.ino
// Zeigt die automatische Firmware-Erkennung und adaptive Verhaltensweisen.

#include "OpenLogDriver.h"

// Definiere die Pins für die serielle Kommunikation und den Reset
#define OPENLOG_RX_PIN 21 // ESP32 GPIO21 (RX1) -> OpenLog TXO
#define OPENLOG_TX_PIN 20 // ESP32 GPIO20 (TX1) -> OpenLog RXI
#define OPENLOG_DTR_GRN_PIN 10 // ESP32 GPIO10 -> OpenLog GRN (Reset)

// Erstelle eine Instanz des OpenLogTreiber
// Hier könnten Sie OpenLogFirmwareType::OPENLOG_LIGHT oder OPENLOG_MINIMAL vorgeben,
// wenn Sie wissen, welche Firmware installiert ist, um die Erkennung zu lenken.
OpenLogDriver openLog(Serial1, OPENLOG_DTR_GRN_PIN, OpenLogFirmwareType::UNKNOWN);

void setup() {
    Serial.begin(115200); // Serielle Kommunikation für Debug-Ausgaben
    while (!Serial) {
        ; // Warte auf Serial Monitor
    }
    Serial.println("Starte Firmware-Erkennungs Beispiel...");

    OpenLogFirmwareType detectedFirmware = openLog.begin(15000); // Längeres Timeout für umfassende Erkennung

    if (detectedFirmware == OpenLogFirmwareType::UNKNOWN) {
        Serial.println("FEHLER: OpenLog konnte nicht initialisiert oder Firmware erkannt werden!");
        while (true) {
            delay(100);
        }
    }

    Serial.print("OpenLog ist online. Erkannte Firmware: ");
    switch (detectedFirmware) {
        case OpenLogFirmwareType::OPENLOG_STANDARD: Serial.println("OpenLog Standard"); break;
        case OpenLogFirmwareType::OPENLOG_LIGHT:    Serial.println("OpenLog Light");    break;
        case OpenLogFirmwareType::OPENLOG_MINIMAL:  Serial.println("OpenLog Minimal");  break;
        default: Serial.println("Unbekannt"); break;
    }
    Serial.print("Aktuelle Baudrate: ");
    Serial.println(openLog.getCurrentBaudRate());
    Serial.print("Aktueller Modus: ");
    switch (openLog.getOpenLogMode()) {
        case OpenLogMode::LOGGING_MODE: Serial.println("Logging Modus"); break;
        case OpenLogMode::COMMAND_MODE: Serial.println("Befehlsmodus"); break;
        case OpenLogMode::ERROR_MODE:   Serial.println("Fehlermodus");   break;
        default: Serial.println("Unbekannt"); break;
    }

    // Versuche, Befehle zu senden, basierend auf der erkannten Firmware
    if (detectedFirmware == OpenLogFirmwareType::OPENLOG_STANDARD) {
        Serial.println("\n--- Teste Befehlsmodus-Funktionen (OpenLog Standard) ---");
        String version;
        if (openLog.queryFirmwareVersion(version) == OpenLogStatus::OK) {
            Serial.print("Firmware Version: ");
            Serial.println(version);
        } else {
            Serial.println("Fehler beim Abfragen der Firmware-Version.");
        }

        String diskInfo;
        if (openLog.sendCommand("disk", diskInfo) == OpenLogStatus::OK) {
            Serial.println("Disk Info:");
            Serial.println(diskInfo);
        } else {
            Serial.println("Fehler beim Abfragen der Disk-Informationen.");
        }

        // Beispiel: Baudrate ändern (würde einen Reset erfordern, um wirksam zu werden)
        // if (openLog.setBaudRate(57600) == OpenLogStatus::OK) {
        //     Serial.println("Baudrate auf 57600 gesetzt. Bitte OpenLog resetten!");
        //     // openLog.resetOpenLogHardware(); // Würde hier einen Reset auslösen
        // } else {
        //     Serial.println("Fehler beim Setzen der Baudrate.");
        // }

        // Zurück in den Logging-Modus wechseln, um Daten zu schreiben
        if (openLog.exitCommandMode() == OpenLogStatus::OK) {
            Serial.println("Erfolgreich in den Logging-Modus gewechselt.");
        } else {
            Serial.println("Fehler beim Wechsel in den Logging-Modus.");
        }
    } else {
        Serial.println("\n--- Befehlsmodus-Funktionen nicht verfügbar (Logging-only Firmware) ---");
        Serial.println("Diese Firmware-Version unterstützt keine Befehle zur Laufzeit.");
        String version;
        if (openLog.queryFirmwareVersion(version) == OpenLogStatus::NOT_SUPPORTED) {
            Serial.println("queryFirmwareVersion(): Nicht unterstützt (korrekt).");
        }
    }
}

void loop() {
    // Im Loop können weiterhin Daten geloggt werden, unabhängig vom Firmware-Typ
    static unsigned long lastLogTime = 0;
    const unsigned long logInterval = 5000; // Alle 5 Sekunden loggen

    if (millis() - lastLogTime >= logInterval) {
        lastLogTime = millis();
        String logData = "Loop Timestamp: " + String(millis());
        if (openLog.write(logData + "\r\n") == OpenLogStatus::OK) {
            Serial.println("Daten im Loop geloggt.");
        } else {
            Serial.println("Fehler beim Loggen im Loop.");
        }
    }
}
