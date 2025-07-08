
# OpenLogDriver - ATTN: WORK IN PROGRESS - README FILE INVALID


## 1. OpenLogDriver/src/OpenLogDriver.h

````C++

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
````


## 2. OpenLogDriver/src/OpenLogDriver.cpp

C++

// OpenLogDriver.cpp

#include "OpenLogDriver.h"

// Baudraten, die beim Sondieren versucht werden sollen (absteigend, da höhere Raten schneller fehlschlagen)
const long BAUD_RATES = {115200, 57600, 38400, 19200, 9600, 4800, 2400};
const int NUM_BAUD_RATES = sizeof(BAUD_RATES) / sizeof(BAUD_RATES);

// Standard-Escape-Sequenz für OpenLog (CTRL+z, 3x)
const byte DEFAULT_ESCAPE_CHAR = 26; // ASCII for CTRL+z
const byte DEFAULT_ESCAPE_COUNT = 3;

OpenLogDriver::OpenLogDriver(HardwareSerial& serialPort, int dtrGrnPin, OpenLogFirmwareType expectedFirmware)
    : _serial(serialPort),
      _dtrGrnPin(dtrGrnPin),
      _firmwareType(OpenLogFirmwareType::UNKNOWN),
      _currentMode(OpenLogMode::UNKNOWN),
      _currentBaudRate(0),
      _expectedFirmware(expectedFirmware),
      _voltageAdcPin(-1),
      _voltageDividerRatio(1.0),
      _brownoutThreshold(0.0),
      _voltageMonitoringEnabled(false) {
    // Initialisiere den DTR/GRN-Pin als Ausgang
    pinMode(_dtrGrnPin, OUTPUT);
    digitalWrite(_dtrGrnPin, HIGH); // DTR/GRN standardmäßig HIGH halten (nicht resetten)
}

OpenLogFirmwareType OpenLogDriver::begin(unsigned long timeoutMs) {
    Serial.println("OpenLogDriver: Initialisiere OpenLog...");

    // Phase 1: Hardware-Reset
    Serial.println("OpenLogDriver: Führe Hardware-Reset durch...");
    resetOpenLogHardware();
    delay(500); // Warte, bis OpenLog neu gestartet ist

    // Phase 2: Baudraten-Sondierung
    Serial.println("OpenLogDriver: Sondiere Baudrate...");
    if (!probeBaudRate(timeoutMs)) {
        Serial.println("OpenLogDriver: Fehler: Baudrate konnte nicht erkannt werden.");
        _firmwareType = OpenLogFirmwareType::UNKNOWN;
        _currentMode = OpenLogMode::ERROR_MODE;
        return _firmwareType;
    }
    Serial.print("OpenLogDriver: Baudrate erkannt: ");
    Serial.println(_currentBaudRate);

    // Phase 3: Firmware-Erkennung
    Serial.println("OpenLogDriver: Erkenne Firmware-Typ...");
    _firmwareType = detectFirmware(timeoutMs);
    Serial.print("OpenLogDriver: Firmware-Typ erkannt: ");
    switch (_firmwareType) {
        case OpenLogFirmwareType::OPENLOG_STANDARD: Serial.println("OPENLOG_STANDARD"); break;
        case OpenLogFirmwareType::OPENLOG_LIGHT:    Serial.println("OPENLOG_LIGHT");    break;
        case OpenLogFirmwareType::OPENLOG_MINIMAL:  Serial.println("OPENLOG_MINIMAL");  break;
        case OpenLogFirmwareType::UNKNOWN:          Serial.println("UNKNOWN");          break;
    }

    // Phase 4: Konfigurationsprüfung und -anpassung (nur für Standard-Firmware)
    if (_firmwareType == OpenLogFirmwareType::OPENLOG_STANDARD) {
        Serial.println("OpenLogDriver: Konfiguriere OpenLog (Standard-Firmware)...");
        // Versuche, in den Befehlsmodus zu wechseln, falls nicht schon geschehen
        if (_currentMode!= OpenLogMode::COMMAND_MODE) {
            if (enterCommandMode()!= OpenLogStatus::OK) {
                Serial.println("OpenLogDriver: Warnung: Konnte nicht in den Befehlsmodus wechseln.");
                // Trotzdem fortfahren, aber Befehlsfunktionen sind möglicherweise nicht verfügbar
            }
        }

        if (_currentMode == OpenLogMode::COMMAND_MODE) {
            // Setze empfohlene Einstellungen für robuste Kommunikation
            OpenLogConfig desiredConfig;
            desiredConfig.baudRate = _currentBaudRate; // Behalte die erkannte Baudrate bei
            desiredConfig.escapeChar = DEFAULT_ESCAPE_CHAR;
            desiredConfig.escapeCharCount = DEFAULT_ESCAPE_COUNT;
            desiredConfig.mode = 0; // New Log Mode (Standard für Logging)
            desiredConfig.verbose = 0; // Fehler: Nur '!'
            desiredConfig.echo = 0;    // Kein Echo im Befehlsmodus
            desiredConfig.ignoreRX = 0; // Notfall-Reset aktiviert lassen

            if (setConfig(desiredConfig) == OpenLogStatus::OK) {
                Serial.println("OpenLogDriver: Konfiguration erfolgreich angewendet.");
                // Nach Konfigurationsänderungen ist ein Reset oft nötig
                Serial.println("OpenLogDriver: Führe erneuten Hardware-Reset durch, um Konfiguration anzuwenden...");
                resetOpenLogHardware();
                delay(500); // Warte auf Neustart
                // Erneute Baudraten-Sondierung, um den neuen Zustand zu bestätigen
                if (!probeBaudRate(timeoutMs)) {
                    Serial.println("OpenLogDriver: Fehler: Baudrate nach Konfiguration nicht bestätigt.");
                    _firmwareType = OpenLogFirmwareType::UNKNOWN;
                    _currentMode = OpenLogMode::ERROR_MODE;
                    return _firmwareType;
                }
            } else {
                Serial.println("OpenLogDriver: Warnung: Konfiguration konnte nicht angewendet werden.");
            }
            // Nach der Konfiguration sollte der OpenLog im Logging-Modus sein
            _currentMode = OpenLogMode::LOGGING_MODE;
        }
    } else {
        Serial.println("OpenLogDriver: Firmware ist Logging-only, keine Konfiguration über Befehle möglich.");
        _currentMode = OpenLogMode::LOGGING_MODE; // Gehe davon aus, dass es im Logging-Modus ist
    }

    Serial.println("OpenLogDriver: Initialisierung abgeschlossen.");
    return _firmwareType;
}

void OpenLogDriver::resetOpenLogHardware() {
    digitalWrite(_dtrGrnPin, LOW); // DTR/GRN auf LOW ziehen für Reset
    delay(100); // Kurz halten
    digitalWrite(_dtrGrnPin, HIGH); // DTR/GRN wieder HIGH (oder als INPUT_PULLUP konfigurieren)
    // Für ESP32 GPIOs ist HIGH oft ausreichend, um den Pin freizugeben.
    // Alternativ: pinMode(_dtrGrnPin, INPUT_PULLUP);
    delay(1000); // Warte, bis der OpenLog den Reset verarbeitet hat und startet
    clearSerialBuffer(); // Seriellen Puffer leeren, um alte Daten zu entfernen
}

bool OpenLogDriver::probeBaudRate(unsigned long timeoutMs) {
    String response;
    unsigned long startTime = millis();

    for (int i = 0; i < NUM_BAUD_RATES; ++i) {
        _currentBaudRate = BAUD_RATES[i];
        _serial.begin(_currentBaudRate);
        _serial.setTimeout(timeoutMs);
        clearSerialBuffer(); // Puffer vor jedem Versuch leeren

        Serial.print("OpenLogDriver: Versuche Baudrate: ");
        Serial.println(_currentBaudRate);

        // Sende eine harmlose Sequenz, um den OpenLog zu provozieren
        // 3x CTRL+z ist die Standard-Escape-Sequenz, die eine Reaktion hervorrufen sollte
        // auch wenn der OpenLog im Logging-Modus ist.
        _serial.write(DEFAULT_ESCAPE_CHAR);
        _serial.write(DEFAULT_ESCAPE_CHAR);
        _serial.write(DEFAULT_ESCAPE_CHAR);
        _serial.print("\r"); // Sende Carriage Return, um Befehl abzuschließen

        response = readLine(timeoutMs / NUM_BAUD_RATES); // Kürzere Timeouts pro Versuch
        if (response.length() > 0) {
            // Überprüfe auf Start-Prompts
            if (response.indexOf("12<")!= -1 |

| response.indexOf("12>")!= -1) {
                _currentMode = parseStartupPrompt(response);
                return true; // Baudrate erfolgreich erkannt
            }
            // Manchmal kommt nur der Prompt ohne 12, wenn schon im Modus
            if (response.indexOf("<")!= -1) {
                _currentMode = OpenLogMode::LOGGING_MODE;
                return true;
            }
            if (response.indexOf(">")!= -1) {
                _currentMode = OpenLogMode::COMMAND_MODE;
                return true;
            }
        }
        // Wenn Timeout erreicht ist, aber noch Zeit übrig ist, versuche die nächste Baudrate
        if (millis() - startTime > timeoutMs) {
            break; // Gesamttimeout erreicht
        }
    }
    return false; // Keine Baudrate erkannt
}

OpenLogFirmwareType OpenLogDriver::detectFirmware(unsigned long timeoutMs) {
    String response;
    OpenLogStatus status;

    // Zuerst versuchen, in den Befehlsmodus zu wechseln und? Befehl zu senden
    status = enterCommandMode();
    if (status == OpenLogStatus::OK) {
        status = queryFirmwareVersion(response);
        if (status == OpenLogStatus::OK && response.length() > 0) {
            // Wenn wir eine Versionsnummer bekommen, ist es die Standard-Firmware
            // Optional: Hier könnte man die Version parsen, um spezifische Features zu erkennen
            // Für diesen Treiber reicht die Unterscheidung der 3 Haupt-Firmware-Typen
            _currentMode = OpenLogMode::COMMAND_MODE; // Bleibt im Befehlsmodus nach Abfrage
            return OpenLogFirmwareType::OPENLOG_STANDARD;
        }
    }

    // Wenn der Befehlsmodus nicht erreicht wurde oder der? Befehl fehlschlug,
    // ist es wahrscheinlich OpenLog_Light oder OpenLog_Minimal.
    // Beide haben keinen Befehlsmodus und starten im Logging-Modus.
    // Wir können sie nicht direkt unterscheiden, ohne die config.txt zu manipulieren
    // oder den Code zu kennen.
    _currentMode = OpenLogMode::LOGGING_MODE; // Gehe davon aus, dass es im Logging-Modus ist

    if (_expectedFirmware!= OpenLogFirmwareType::UNKNOWN && _expectedFirmware!= OpenLogFirmwareType::OPENLOG_STANDARD) {
        return _expectedFirmware; // Verwende die vom Benutzer vorgegebene Firmware
    } else {
        // Standardmäßig OpenLog_Light annehmen, da es flexibler ist als Minimal
        return OpenLogFirmwareType::OPENLOG_LIGHT;
    }
}

OpenLogMode OpenLogDriver::parseStartupPrompt(const String& prompt) {
    if (prompt.indexOf("12<")!= -1) {
        return OpenLogMode::LOGGING_MODE;
    } else if (prompt.indexOf("12>")!= -1) {
        return OpenLogMode::COMMAND_MODE;
    } else if (prompt.indexOf("<")!= -1) { // Fallback, falls "12" fehlt
        return OpenLogMode::LOGGING_MODE;
    } else if (prompt.indexOf(">")!= -1) { // Fallback, falls "12" fehlt
        return OpenLogMode::COMMAND_MODE;
    }
    return OpenLogMode::UNKNOWN;
}

void OpenLogDriver::clearSerialBuffer() {
    while (_serial.available()) {
        _serial.read();
    }
}

String OpenLogDriver::readLine(unsigned long timeoutMs) {
    String response = "";
    unsigned long startTime = millis();
    while (millis() - startTime < timeoutMs) {
        if (_serial.available()) {
            char c = _serial.read();
            response += c;
            if (c == '\n' |

| c == '\r') { // Suche nach Zeilenende
                // Optional: Warte auf das zweite Zeichen des Zeilenumbruchs (CRLF)
                if (c == '\r' && _serial.peek() == '\n') {
                    _serial.read(); // Lese das '\n'
                }
                return response;
            }
        }
    }
    return response; // Rückgabe, auch wenn Timeout erreicht und kein Zeilenende gefunden
}

OpenLogStatus OpenLogDriver::sendAndExpect(const String& data, const String& expectedPrompt, String& response, unsigned long timeoutMs) {
    clearSerialBuffer();
    _serial.print(data);
    _serial.print("\r"); // Befehle benötigen Carriage Return

    response = readLine(timeoutMs);

    if (response.length() == 0) {
        return OpenLogStatus::TIMEOUT;
    }
    if (response.indexOf(expectedPrompt)!= -1) {
        return OpenLogStatus::OK;
    }
    if (response.indexOf("!")!= -1) { // Generisches Fehlersignal
        return OpenLogStatus::ERROR;
    }
    // Weitere spezifische Fehlerprüfungen könnten hier hinzugefügt werden
    return OpenLogStatus::ERROR; // Unerwartete Antwort
}

OpenLogStatus OpenLogDriver::write(const String& data) {
    if (_firmwareType == OpenLogFirmwareType::UNKNOWN |

| _currentMode == OpenLogMode::ERROR_MODE) {
        return OpenLogStatus::ERROR; // OpenLog nicht initialisiert oder im Fehlerzustand
    }

    // Wenn Standard-Firmware und nicht im Logging-Modus, versuche zu wechseln
    if (_firmwareType == OpenLogFirmwareType::OPENLOG_STANDARD && _currentMode!= OpenLogMode::LOGGING_MODE) {
        Serial.println("OpenLogDriver: Versuche, in den Logging-Modus zu wechseln...");
        if (exitCommandMode()!= OpenLogStatus::OK) {
            Serial.println("OpenLogDriver: Fehler: Konnte nicht in den Logging-Modus wechseln.");
            return OpenLogStatus::ERROR;
        }
    }

    // Daten senden
    _serial.print(data);
    // Optional: Kleine Verzögerung nach großen Datenmengen, um Pufferüberlauf zu vermeiden
    // delay(15); // Nur bei sehr hohen Raten und langen Strings nötig [1]
    return OpenLogStatus::OK;
}

OpenLogStatus OpenLogDriver::write(const char* data) {
    return write(String(data));
}

OpenLogStatus OpenLogDriver::write(byte data) {
    if (_firmwareType == OpenLogFirmwareType::UNKNOWN |

| _currentMode == OpenLogMode::ERROR_MODE) {
        return OpenLogStatus::ERROR;
    }
    if (_firmwareType == OpenLogFirmwareType::OPENLOG_STANDARD && _currentMode!= OpenLogMode::LOGGING_MODE) {
        if (exitCommandMode()!= OpenLogStatus::OK) {
            return OpenLogStatus::ERROR;
        }
    }
    _serial.write(data);
    return OpenLogStatus::OK;
}

OpenLogStatus OpenLogDriver::enterCommandMode() {
    if (_firmwareType!= OpenLogFirmwareType::OPENLOG_STANDARD) {
        return OpenLogStatus::NOT_SUPPORTED;
    }
    if (_currentMode == OpenLogMode::COMMAND_MODE) {
        return OpenLogStatus::OK; // Bereits im Befehlsmodus
    }

    Serial.println("OpenLogDriver: Sende Escape-Sequenz für Befehlsmodus...");
    clearSerialBuffer();
    for (int i = 0; i < DEFAULT_ESCAPE_COUNT; ++i) {
        _serial.write(DEFAULT_ESCAPE_CHAR);
        delay(10); // Kleine Pause zwischen den Escape-Zeichen
    }
    _serial.print("\r"); // Optional, um den Puffer zu leeren und eine Reaktion zu provozieren

    String response = readLine(1000); // Warte auf den Prompt
    if (response.indexOf(">")!= -1) {
        _currentMode = OpenLogMode::COMMAND_MODE;
        Serial.println("OpenLogDriver: Erfolgreich in den Befehlsmodus gewechselt.");
        return OpenLogStatus::OK;
    } else if (response.indexOf("<")!= -1) {
        // Immer noch im Logging-Modus, Escape-Sequenz wurde nicht erkannt
        Serial.println("OpenLogDriver: Warnung: OpenLog blieb im Logging-Modus.");
        return OpenLogStatus::ERROR;
    }
    Serial.println("OpenLogDriver: Fehler: Keine erwartete Antwort nach Escape-Sequenz.");
    return OpenLogStatus::TIMEOUT;
}

OpenLogStatus OpenLogDriver::exitCommandMode() {
    if (_firmwareType!= OpenLogFirmwareType::OPENLOG_STANDARD) {
        return OpenLogStatus::NOT_SUPPORTED;
    }
    if (_currentMode == OpenLogMode::LOGGING_MODE) {
        return OpenLogStatus::OK; // Bereits im Logging-Modus
    }

    // Sende einen Befehl, der in den Logging-Modus wechselt (z.B. 'new' oder 'append')
    // Hier verwenden wir 'new' mit einem Dummy-Dateinamen, um in den Logging-Modus zu wechseln.
    // Dies erstellt eine neue Datei und wechselt in den Logging-Modus.
    Serial.println("OpenLogDriver: Versuche, Befehlsmodus zu verlassen (new log.txt)...");
    String response;
    OpenLogStatus status = sendCommand("new log.txt", response, 2000); // Längeres Timeout für Dateierstellung
    if (status == OpenLogStatus::OK |

| response.indexOf("<")!= -1) {
        _currentMode = OpenLogMode::LOGGING_MODE;
        Serial.println("OpenLogDriver: Erfolgreich in den Logging-Modus gewechselt.");
        return OpenLogStatus::OK;
    }
    Serial.println("OpenLogDriver: Fehler: Konnte Befehlsmodus nicht verlassen.");
    return OpenLogStatus::ERROR;
}

OpenLogStatus OpenLogDriver::sendCommand(const String& command, String& response, unsigned long timeoutMs) {
    if (_firmwareType!= OpenLogFirmwareType::OPENLOG_STANDARD) {
        return OpenLogStatus::NOT_SUPPORTED;
    }
    if (_currentMode!= OpenLogMode::COMMAND_MODE) {
        // Versuche, in den Befehlsmodus zu wechseln, wenn nicht bereits dort
        if (enterCommandMode()!= OpenLogStatus::OK) {
            Serial.println("OpenLogDriver: Fehler: Konnte nicht in den Befehlsmodus für sendCommand wechseln.");
            return OpenLogStatus::ERROR;
        }
    }

    Serial.print("OpenLogDriver: Sende Befehl: ");
    Serial.println(command);
    OpenLogStatus status = sendAndExpect(command, ">", response, timeoutMs); // Erwarte '>' als Prompt
    if (status == OpenLogStatus::OK) {
        // Entferne den Prompt aus der Antwort, falls vorhanden
        int promptPos = response.indexOf(">");
        if (promptPos!= -1) {
            response = response.substring(0, promptPos);
        }
        response.trim(); // Leerzeichen entfernen
    }
    return status;
}

OpenLogStatus OpenLogDriver::queryFirmwareVersion(String& version) {
    if (_firmwareType!= OpenLogFirmwareType::OPENLOG_STANDARD) {
        return OpenLogStatus::NOT_SUPPORTED;
    }
    String response;
    OpenLogStatus status = sendCommand("?", response, 1000);
    if (status == OpenLogStatus::OK) {
        version = response;
        return OpenLogStatus::OK;
    }
    return status;
}

OpenLogStatus OpenLogDriver::setBaudRate(long baud) {
    if (_firmwareType!= OpenLogFirmwareType::OPENLOG_STANDARD) {
        return OpenLogStatus::NOT_SUPPORTED;
    }
    String response;
    OpenLogStatus status = sendCommand("baud " + String(baud), response, 1000);
    if (status == OpenLogStatus::OK) {
        // Baudrate-Änderung erfordert einen Reset, um wirksam zu werden
        _currentBaudRate = baud; // Aktualisiere interne Baudrate
        Serial.println("OpenLogDriver: Baudrate gesetzt. Bitte OpenLog resetten, damit die Einstellung wirksam wird.");
        return OpenLogStatus::OK;
    }
    return status;
}

OpenLogStatus OpenLogDriver::setMode(byte mode) {
    if (_firmwareType!= OpenLogFirmwareType::OPENLOG_STANDARD) {
        return OpenLogStatus::NOT_SUPPORTED;
    }
    if (mode > 2) return OpenLogStatus::INVALID_PARAMETER;

    String response;
    OpenLogStatus status = sendCommand("set " + String(mode), response, 1000);
    if (status == OpenLogStatus::OK) {
        Serial.println("OpenLogDriver: Modus gesetzt. Bitte OpenLog resetten, damit die Einstellung wirksam wird.");
        return OpenLogStatus::OK;
    }
    return status;
}

OpenLogStatus OpenLogDriver::setVerbose(bool on) {
    if (_firmwareType!= OpenLogFirmwareType::OPENLOG_STANDARD) {
        return OpenLogStatus::NOT_SUPPORTED;
    }
    String state = on? "on" : "off";
    String response;
    return sendCommand("verb " + state, response, 1000);
}

OpenLogStatus OpenLogDriver::setEcho(bool on) {
    if (_firmwareType!= OpenLogFirmwareType::OPENLOG_STANDARD) {
        return OpenLogStatus::NOT_SUPPORTED;
    }
    String state = on? "on" : "off";
    String response;
    return sendCommand("echo " + state, response, 1000);
}

OpenLogStatus OpenLogDriver::syncData() {
    if (_firmwareType == OpenLogFirmwareType::UNKNOWN |

| _currentMode == OpenLogMode::ERROR_MODE) {
        return OpenLogStatus::ERROR;
    }

    // Für Standard-Firmware: Wechsel in den Befehlsmodus, sync senden, zurück in Logging
    if (_firmwareType == OpenLogFirmwareType::OPENLOG_STANDARD) {
        OpenLogMode previousMode = _currentMode;
        if (enterCommandMode()!= OpenLogStatus::OK) {
            Serial.println("OpenLogDriver: Warnung: Konnte nicht in den Befehlsmodus für sync wechseln.");
            return OpenLogStatus::ERROR;
        }
        String response;
        OpenLogStatus status = sendCommand("sync", response, 1000);
        if (previousMode == OpenLogMode::LOGGING_MODE) {
            exitCommandMode(); // Versuche, in den vorherigen Modus zurückzukehren
        }
        return status;
    } else {
        // Für Light/Minimal: Einfach 'sync' senden, da kein Befehlsmodus
        // Der OpenLog_Light/Minimal reagiert nicht mit Prompts auf sync,
        // aber der Befehl wird intern verarbeitet.
        _serial.print("sync\r");
        delay(500); // Gib OpenLog Zeit zum Verarbeiten
        clearSerialBuffer(); // Puffer leeren
        return OpenLogStatus::OK;
    }
}

bool OpenLogDriver::isSDCardOK() {
    // Basierend auf dem initialen Start-Prompt
    if (_currentMode == OpenLogMode::ERROR_MODE) {
        return false; // SD-Kartenfehler wurde beim Start erkannt
    }
    // Für Standard-Firmware, kann 'disk' Befehl verwenden
    if (_firmwareType == OpenLogFirmwareType::OPENLOG_STANDARD) {
        String response;
        OpenLogStatus status = sendCommand("disk", response, 2000);
        if (status == OpenLogStatus::OK && response.indexOf("Card Size:")!= -1) {
            return true; // Erfolgreiche Antwort vom 'disk' Befehl
        }
        return false;
    }
    // Für Light/Minimal, verlassen wir uns auf den initialen Start-Prompt '12<'
    // und die Annahme, dass der OpenLog nicht im ERROR_MODE ist.
    return true;
}

OpenLogStatus OpenLogDriver::getConfig(OpenLogConfig& config) {
    if (_firmwareType!= OpenLogFirmwareType::OPENLOG_STANDARD) {
        return OpenLogStatus::NOT_SUPPORTED;
    }
    String response;
    OpenLogStatus status = sendCommand("?", response, 1000); // '?' zeigt auch Konfig an
    if (status == OpenLogStatus::OK) {
        // Hier müsste die Antwort geparst werden, um die config.txt-Werte zu extrahieren.
        // Dies ist komplex und würde eine detaillierte String-Parsing-Logik erfordern.
        // Für dieses Beispiel wird nur ein Dummy-Wert gesetzt.
        // Eine vollständige Implementierung würde Regex oder String-Manipulationen nutzen.
        config.baudRate = _currentBaudRate; // Annahme
        config.escapeChar = DEFAULT_ESCAPE_CHAR;
        config.escapeCharCount = DEFAULT_ESCAPE_COUNT;
        config.mode = 0; // Annahme
        config.verbose = 0; // Annahme
        config.echo = 0; // Annahme
        config.ignoreRX = 0; // Annahme
        return OpenLogStatus::OK;
    }
    return status;
}

OpenLogStatus OpenLogDriver::setConfig(const OpenLogConfig& config) {
    if (_firmwareType!= OpenLogFirmwareType::OPENLOG_STANDARD) {
        return OpenLogStatus::NOT_SUPPORTED;
    }
    OpenLogStatus status;
    String response;

    // Setze Baudrate (erfordert Reset)
    status = sendCommand("baud " + String(config.baudRate), response, 1000);
    if (status!= OpenLogStatus::OK) return status;

    // Setze Escape-Zeichen
    status = sendCommand("escape " + String(config.escapeChar), response, 1000);
    if (status!= OpenLogStatus::OK) return status;

    // Setze Anzahl Escape-Zeichen
    status = sendCommand("esc# " + String(config.escapeCharCount), response, 1000);
    if (status!= OpenLogStatus::OK) return status;

    // Setze Modus (erfordert Reset)
    status = sendCommand("set " + String(config.mode), response, 1000);
    if (status!= OpenLogStatus::OK) return status;

    // Setze Verbose-Modus
    status = sendCommand("verb " + String(config.verbose), response, 1000);
    if (status!= OpenLogStatus::OK) return status;

    // Setze Echo-Modus
    status = sendCommand("echo " + String(config.echo), response, 1000);
    if (status!= OpenLogStatus::OK) return status;

    // Setze ignoreRX
    status = sendCommand("ignoreRX " + String(config.ignoreRX), response, 1000);
    if (status!= OpenLogStatus::OK) return status;

    // Aktualisiere interne Konfiguration
    _currentConfig = config;
    return OpenLogStatus::OK;
}

void OpenLogDriver::setVoltageMonitoring(int adcPin, float voltageDividerRatio, float brownoutThreshold) {
    _voltageAdcPin = adcPin;
    _voltageDividerRatio = voltageDividerRatio;
    _brownoutThreshold = brownoutThreshold;
    _voltageMonitoringEnabled = true;
    // ESP32 ADC Setup (optional, kann auch im Haupt-Sketch erfolgen)
    // analogReadResolution(12); // 12-bit Auflösung
    // analogSetAttenuation(ADC_11db); // Voller Bereich (0-3.3V)
}

OpenLogStatus OpenLogDriver::checkPowerStatus() {
    if (!_voltageMonitoringEnabled |

| _voltageAdcPin == -1) {
        return OpenLogStatus::NOT_SUPPORTED; // Spannungsüberwachung nicht konfiguriert
    }

    // ADC-Wert lesen
    int adcRaw = analogRead(_voltageAdcPin);

    // Umrechnung in Spannung (angenommen 3.3V Referenz für ESP32 ADC)
    // ESP32 ADC ist nicht linear, dies ist eine vereinfachte Berechnung.
    // Eine präzisere Kalibrierung wäre hier ideal.
    float voltage = (float)adcRaw / 4095.0 * 3.3 * _voltageDividerRatio; // 4095 für 12-bit ADC

    Serial.print("OpenLogDriver: Gemessene Spannung: ");
    Serial.print(voltage);
    Serial.println("V");

    if (voltage < _brownoutThreshold) {
        return OpenLogStatus::BROWNOUT_IMMINENT;
    } else if (voltage < (_brownoutThreshold * 1.1)) { // Z.B. 10% über Schwellenwert als Warnung
        return OpenLogStatus::LOW_VOLTAGE_WARNING;
    }
    return OpenLogStatus::OK;
}

3. OpenLogDriver/examples/Example_BasicLogging.ino

C++

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

4. OpenLogDriver/examples/Example_FirmwareDetection.ino

C++

// Example_FirmwareDetection.ino
// Zeigt die automatische Firmware-Erkennung und adaptive Verhaltensweisen.

#include "OpenLogDriver.h"

// Definiere die Pins für die serielle Kommunikation und den Reset
#define OPENLOG_RX_PIN 21 // ESP32 GPIO21 (RX1) -> OpenLog TXO
#define OPENLOG_TX_PIN 20 // ESP32 GPIO20 (TX1) -> OpenLog RXI
#define OPENLOG_DTR_GRN_PIN 10 // ESP33 GPIO10 -> OpenLog GRN (Reset)

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

5. OpenLogDriver/examples/Example_VoltageMonitoring.ino

C++

// Example_VoltageMonitoring.ino
// Zeigt die Spannungsüberwachung und Reaktion auf niedrige Batteriespannung.

#include "OpenLogDriver.h"

// Definiere die Pins für die serielle Kommunikation und den Reset
#define OPENLOG_RX_PIN 21 // ESP32 GPIO21 (RX1) -> OpenLog TXO
#define OPENLOG_TX_PIN 20 // ESP32 GPIO20 (TX1) -> OpenLog RXI
#define OPENLOG_DTR_GRN_PIN 10 // ESP33 GPIO10 -> OpenLog GRN (Reset)

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

6. OpenLogDriver/LICENSE

MIT License

Copyright (c) 2025 Thomas Walloschke

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

7. OpenLogDriver/README.md

ESP32-C3 OpenLog Robust Driver

A robust Arduino library for ESP32-C3 to reliably communicate with SparkFun OpenLog data loggers, designed for autonomous logging applications in challenging environments. This driver automatically detects OpenLog firmware variants, handles unknown baud rates, and includes integrated power monitoring to prevent data loss due to brownouts.

## Motivation

For a detailed technical analysis of the problem, the proposed solutions, and the design philosophy behind this driver, please refer to the(TECHNICAL.md) document. This document provides an in-depth report on the challenges of OpenLog communication and the strategies implemented for maximum robustness.

### Features

    Automatic Firmware Detection: Identifies OpenLog Standard, OpenLog_Light, and OpenLog_Minimal firmware variants.

    Adaptive Baud Rate Negotiation: Automatically probes and synchronizes with the OpenLog's current baud rate.

    Hardware Reset Control: Utilizes the DTR/GRN pin for deterministic OpenLog resets.

    Intelligent Configuration: For OpenLog Standard firmware, automatically configures optimal settings (echo off, verbose off, logging mode) for embedded use.

    Integrated Power Monitoring: Detects low voltage conditions and imminent brownouts, allowing for graceful logging termination and data synchronization to prevent corruption.

    Robust Data Logging: Provides methods for reliable data writing and buffer synchronization.

    SD Card Health Check: Basic checks for SD card status.

    Comprehensive API: A well-defined C++ class for easy integration into ESP32 projects.

### Hardware Setup

To use this library, connect your ESP32-C3 to the SparkFun OpenLog as follows:

    ESP32-C3 GPIO21 (RX1) -> OpenLog TXO

    ESP32-C3 GPIO20 (TX1) -> OpenLog RXI

    ESP32-C3 GPIO10 -> OpenLog GRN (Reset Pin)

    ESP32-C3 3V3 -> OpenLog VCC

    ESP32-C3 GND -> OpenLog GND

#### For voltage monitoring:

    Connect a voltage divider network from your battery/VCC to an ESP32-C3 ADC pin (e.g., GPIO36). Ensure the voltage divider scales the input voltage to be within the ESP32's ADC input range (typically 0-3.3V).

### Installation

    Download: Download this repository as a .zip file.

    Arduino IDE: Open your Arduino IDE.

    Add Library: Go to Sketch > Include Library > Add.ZIP Library... and select the downloaded .zip file.

    Examples: The example sketches will be available under File > Examples > ESP32-C3 OpenLog Robust Driver.

## Example Usage

Refer to the examples provided in the examples/ directory:

    Example_BasicLogging.ino: Demonstrates basic data logging.

    Example_FirmwareDetection.ino: Shows how the driver automatically detects the OpenLog firmware type and adapts its behavior.

    Example_VoltageMonitoring.ino: Illustrates the integrated power monitoring feature and graceful shutdown.

## Author

Thomas Walloschke artkeller@gmx.de

## License

This project is licensed under the MIT License. See the(LICENSE) file for details.

## Contributors

No contributors at this time.

## Disclaimer

This software is provided "as is" without any warranty, express or implied. The author makes no guarantee for failsafe applications. Use at your own risk.

## Version

Official Alpha Version 0.1.1
