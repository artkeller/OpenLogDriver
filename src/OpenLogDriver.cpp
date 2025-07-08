// OpenLogDriver.cpp

#include "OpenLogDriver.h"

// Baudraten, die beim Sondieren versucht werden sollen (absteigend, da höhere Raten schneller fehlschlagen)
const long BAUD_RATES = {115200, 57600, 38400, 19200, 9600, 4800, 2400};
const int NUM_BAUD_RATES = sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]);

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
            if (response.indexOf("12<")!= -1 || response.indexOf("12>")!= -1) {
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
    if (_firmwareType == OpenLogFirmwareType::UNKNOWN || _currentMode == OpenLogMode::ERROR_MODE) {
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