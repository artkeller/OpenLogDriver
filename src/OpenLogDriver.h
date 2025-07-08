// OpenLogDriver.h

#ifndef OPENLOG_DRIVER_H
#define OPENLOG_DRIVER_H

#include <Arduino.h>
#include <HardwareSerial.h> // Für ESP32 HardwareSerial

// Enum für den erkannten Firmware-Typ
enum class OpenLogFirmwareType {
    UNKNOWN,
    OPENLOG_STANDARD, // OpenLog (Default)
    OPENLOG_LIGHT,    // OpenLog_Light (Logging-only, config.txt)
    OPENLOG_MINIMAL   // OpenLog_Minimal (Logging-only, baudrate hardcoded)
};

// Enum für den aktuellen Betriebsmodus des OpenLog
enum class OpenLogMode {
    UNKNOWN,
    LOGGING_MODE,
    COMMAND_MODE,
    ERROR_MODE // Z.B. SD-Kartenfehler
};

// Enum für den Status von Operationen
enum class OpenLogStatus {
    OK,
    ERROR,
    TIMEOUT,
    NOT_SUPPORTED,
    INVALID_PARAMETER,
    SD_CARD_ERROR,
    FIRMWARE_MISMATCH,
    LOW_VOLTAGE_WARNING, // Neu: Warnung vor niedriger Spannung
    BROWNOUT_IMMINENT    // Neu: Brownout droht
};

// Struktur für OpenLog-Konfigurationseinstellungen
struct OpenLogConfig {
    long baudRate;
    byte escapeChar;
    byte escapeCharCount;
    byte mode; // 0: New Log, 1: Sequential Log, 2: Command Mode
    byte verbose; // 0: off, 1: on
    byte echo;    // 0: off, 1: on
    byte ignoreRX; // 0: enabled, 1: disabled
};

class OpenLogDriver {
public:
    // Konstruktor
    // serialPort: Referenz auf die HardwareSerial-Instanz (z.B. &Serial1)
    // dtrGrnPin: GPIO-Pin für DTR/GRN (Reset) des OpenLog
    // expectedFirmware: Optional, kann eine erwartete Firmware-Version vorgeben,
    //                   um die Erkennung zu beschleunigen oder zu lenken.
    OpenLogDriver(HardwareSerial& serialPort, int dtrGrnPin, OpenLogFirmwareType expectedFirmware = OpenLogFirmwareType::UNKNOWN);

    // Initialisiert den OpenLog-Treiber und versucht, den OpenLog zu synchronisieren.
    // Führt Hardware-Reset, Baudraten-Sondierung und Firmware-Erkennung durch.
    // Gibt den erkannten Firmware-Typ zurück oder UNKNOWN bei Fehler.
    OpenLogFirmwareType begin(unsigned long timeoutMs = 5000);

    // Gibt den aktuell erkannten Firmware-Typ zurück.
    OpenLogFirmwareType getFirmwareType() const { return _firmwareType; }

    // Gibt den aktuellen Betriebsmodus des OpenLog zurück (Logging, Command, Error).
    OpenLogMode getOpenLogMode() const { return _currentMode; }

    // Gibt die aktuell verwendete Baudrate zurück.
    long getCurrentBaudRate() const { return _currentBaudRate; }

    // Schreibt Daten in den OpenLog.
    // Funktioniert nur im LOGGING_MODE.
    // Bei OPENLOG_STANDARD: Versucht ggf. in den Logging-Modus zu wechseln.
    OpenLogStatus write(const String& data);
    OpenLogStatus write(const char* data);
    OpenLogStatus write(byte data);

    // Versucht, in den Befehlsmodus zu wechseln.
    // Nur für OPENLOG_STANDARD Firmware.
    OpenLogStatus enterCommandMode();

    // Versucht, den Befehlsmodus zu verlassen und in den Logging-Modus zu wechseln.
    // Nur für OPENLOG_STANDARD Firmware.
    OpenLogStatus exitCommandMode();

    // Sendet einen Befehl an den OpenLog und wartet auf eine Antwort.
    // Nur für OPENLOG_STANDARD Firmware.
    // command: Der zu sendende Befehl (z.B. "ls", "disk").
    // responseBuffer: Puffer für die Antwort des OpenLog.
    // bufferSize: Größe des Antwortpuffers.
    // timeoutMs: Timeout für die Antwort.
    OpenLogStatus sendCommand(const String& command, String& response, unsigned long timeoutMs = 1000);

    // Fragt die Firmware-Version des OpenLog ab.
    // Nur für OPENLOG_STANDARD Firmware.
    OpenLogStatus queryFirmwareVersion(String& version);

    // Setzt die Baudrate des OpenLog.
    // Nur für OPENLOG_STANDARD Firmware. Erfordert einen Reset des OpenLog,
    // damit die Einstellung wirksam wird.
    OpenLogStatus setBaudRate(long baud);

    // Setzt den Betriebsmodus des OpenLog (New Log, Sequential Log, Command Mode).
    // Nur für OPENLOG_STANDARD Firmware. Erfordert einen Reset des OpenLog.
    OpenLogStatus setMode(byte mode);

    // Setzt den Verbose-Modus des OpenLog (Fehlermeldungen).
    // Nur für OPENLOG_STANDARD Firmware.
    OpenLogStatus setVerbose(bool on);

    // Setzt den Echo-Modus des OpenLog (Zeichenecho im Befehlsmodus).
    // Nur für OPENLOG_STANDARD Firmware.
    OpenLogStatus setEcho(bool on);

    // Synchronisiert den Puffer des OpenLog mit der SD-Karte.
    // Funktioniert in allen Modi, aber besonders wichtig im Logging-Modus.
    OpenLogStatus syncData();

    // Führt einen Hardware-Reset des OpenLog durch.
    void resetOpenLogHardware();

    // Überprüft den SD-Kartenstatus (basierend auf Start-Prompt oder 'disk' Befehl).
    // Gibt true zurück, wenn die SD-Karte als OK erkannt wird.
    bool isSDCardOK();

    // Gibt die aktuellen Konfigurationseinstellungen des OpenLog zurück.
    // Nur für OPENLOG_STANDARD Firmware.
    OpenLogStatus getConfig(OpenLogConfig& config);

    // Setzt die Konfigurationseinstellungen des OpenLog.
    // Nur für OPENLOG_STANDARD Firmware.
    OpenLogStatus setConfig(const OpenLogConfig& config);

    // Neu: Konfiguriert die Spannungsüberwachung.
    // adcPin: GPIO-Pin, an den der Spannungsteiler angeschlossen ist.
    // voltageDividerRatio: Verhältnis des Spannungsteilers (z.B. 0.5 für 1:2 Teiler).
    // brownoutThreshold: Spannungswert (in Volt), unter dem ein Brownout droht.
    void setVoltageMonitoring(int adcPin, float voltageDividerRatio, float brownoutThreshold);

    // Neu: Überprüft den aktuellen Stromversorgungsstatus.
    // Gibt OK, LOW_VOLTAGE_WARNING oder BROWNOUT_IMMINENT zurück.
    OpenLogStatus checkPowerStatus();

private:
    HardwareSerial& _serial;
    int _dtrGrnPin;
    OpenLogFirmwareType _firmwareType;
    OpenLogMode _currentMode;
    long _currentBaudRate;
    OpenLogConfig _currentConfig; // Speichert die zuletzt bekannte Konfiguration
    OpenLogFirmwareType _expectedFirmware; // Vom Benutzer vorgegebene erwartete Firmware

    // Neu: Variablen für die Spannungsüberwachung
    int _voltageAdcPin;
    float _voltageDividerRatio;
    float _brownoutThreshold;
    bool _voltageMonitoringEnabled;

    // Hilfsfunktion zum Lesen einer Zeile vom OpenLog mit Timeout
    String readLine(unsigned long timeoutMs);

    // Hilfsfunktion zum Senden von Daten und Warten auf einen erwarteten Prompt
    OpenLogStatus sendAndExpect(const String& data, const String& expectedPrompt, String& response, unsigned long timeoutMs);

    // Interne Funktion zur Baudraten-Sondierung
    bool probeBaudRate(unsigned long timeoutMs);

    // Interne Funktion zur Firmware-Erkennung
    OpenLogFirmwareType detectFirmware(unsigned long timeoutMs);

    // Interne Funktion zum Parsen des Start-Prompts
    OpenLogMode parseStartupPrompt(const String& prompt);

    // Interne Funktion zum Löschen des seriellen Puffers
    void clearSerialBuffer();
};

#endif // OPENLOG_DRIVER_H