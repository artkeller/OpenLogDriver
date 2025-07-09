// OpenLog.cpp

#include "OpenLogDriver.h"

const long BAUD_RATES[] = {9600, 19200, 38400, 57600, 115200, 4800, 2400};
const int NUM_BAUD_RATES = sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]);
const byte DEFAULT_ESCAPE_CHAR = 26; // CTRL+Z
const byte DEFAULT_ESCAPE_COUNT = 3;

OpenLogDriver::OpenLogDriver(HardwareSerial& serialPort, int rxPin, int txPin, int dtrGrnPin,
                             OpenLogFirmwareType expectedFirmware)
  : _serial(serialPort), _rxPin(rxPin), _txPin(txPin), _dtrGrnPin(dtrGrnPin),
    _expectedFirmware(expectedFirmware) {
  pinMode(_dtrGrnPin, OUTPUT);
  digitalWrite(_dtrGrnPin, HIGH);
}

OpenLogDriver::InitState OpenLogDriver::beginAsync() {
  switch (_initState) {
    case InitState::NOT_STARTED:
      resetOpenLogHardware();
      _initStartTime = millis();
      _initState = InitState::RESETTING;
      break;

    case InitState::RESETTING:
      if (millis() - _initStartTime > 200) { // Nur 200ms warten
        // Serial mit Pins initialisieren
        _serial.begin(9600, SERIAL_8N1, _rxPin, _txPin);
        clearSerialBuffer();
        _initState = InitState::PROBING_BAUD;
        _currentBaudIndex = 0;
        _lastProbeTime = millis();
        _initStartTime = millis(); // Reset für PROBING_BAUD
      }
      break;

    case InitState::PROBING_BAUD:
      // Sende Escape-Sequenz alle 100ms
      if (millis() - _lastProbeTime > 100) {
        for (int i = 0; i < DEFAULT_ESCAPE_COUNT; i++) {
          _serial.write(DEFAULT_ESCAPE_CHAR);
        }
        _serial.write('\r');
        _lastProbeTime = millis();
      }

      if (_serial.available()) {
        String response = readLine(50); // Kurzer Timeout
        if (response.length() > 0) {
          // Hex-Dump der Antwort
          Serial.print("Raw response (hex): ");
          for (int i = 0; i < response.length(); i++) {
            if (i > 0) Serial.print(" ");
            Serial.print("0x");
            Serial.print(static_cast<byte>(response[i]), HEX);
          }
          Serial.println();
          
          // Vereinfachte Erkennung
          if (response.indexOf('>') != -1 || response.indexOf('<') != -1) {
            Serial.println("Valid prompt detected!");
            _currentBaudRate = BAUD_RATES[_currentBaudIndex];
            _initState = InitState::DETECTING_FIRMWARE;
            _initStartTime = millis();
          }
        }
      }

      // Baudratenwechsel nach 300ms ohne Antwort
      if (millis() - _initStartTime > 300) {
        _currentBaudIndex++;
        
        if (_currentBaudIndex >= NUM_BAUD_RATES) {
          _initState = InitState::ERROR;
          break;
        }
        
        _serial.end();
        _serial.begin(BAUD_RATES[_currentBaudIndex], SERIAL_8N1, _rxPin, _txPin);
        clearSerialBuffer();
        _initStartTime = millis();
        _lastProbeTime = millis();
      }
      break;

    case InitState::DETECTING_FIRMWARE:
      if (enterCommandMode() == OpenLogStatus::OK) {
        _firmwareType = OpenLogFirmwareType::OPENLOG_STANDARD;
      } else {
        _firmwareType = (_expectedFirmware != OpenLogFirmwareType::UNKNOWN)
                          ? _expectedFirmware
                          : OpenLogFirmwareType::OPENLOG_LIGHT;
      }
      _initState = InitState::COMPLETE;
      break;

    case InitState::COMPLETE:
    case InitState::ERROR:
      break;
  }

  return _initState;
}

void OpenLogDriver::resetOpenLogHardware() {
  digitalWrite(_dtrGrnPin, LOW);
  delay(20); // Sehr kurzer Reset-Puls
  digitalWrite(_dtrGrnPin, HIGH);
  
  // Vollständiger Reset
  _initState = InitState::NOT_STARTED;
  _currentBaudIndex = 0;
  _currentMode = OpenLogMode::UNKNOWN;
  
  if (_serial) {
    _serial.end();
  }
}

String OpenLogDriver::readLine(unsigned long timeoutMs) {
  String response = "";
  unsigned long startTime = millis();
  while (millis() - startTime < timeoutMs) {
    if (_serial.available()) {
      char c = _serial.read();
      response += c;
      if (c == '\n') {
        return response;
      }
    }
  }
  return response;
}

void OpenLogDriver::clearSerialBuffer() {
  while (_serial.available()) {
    _serial.read();
  }
}

OpenLogMode OpenLogDriver::parseStartupPrompt(const String& prompt) {
  // Vereinfachte Erkennung
  if (prompt.indexOf('<') != -1) {
    return OpenLogMode::LOGGING_MODE;
  }
  if (prompt.indexOf('>') != -1) {
    return OpenLogMode::COMMAND_MODE;
  }
  return OpenLogMode::UNKNOWN;
}

OpenLogStatus OpenLogDriver::sendAndExpect(const String& data, const String& expectedPrompt,
                                           String& response, unsigned long timeoutMs) {
  clearSerialBuffer();
  _serial.print(data);
  _serial.print("\r");

  response = readLine(timeoutMs);

  if (response.length() == 0) {
    return OpenLogStatus::TIMEOUT;
  }
  if (response.indexOf(expectedPrompt) != -1) {
    return OpenLogStatus::OK;
  }
  if (response.indexOf("!") != -1) {
    return OpenLogStatus::ERROR;
  }
  return OpenLogStatus::ERROR;
}

OpenLogStatus OpenLogDriver::write(const String& data) {
  if (_firmwareType == OpenLogFirmwareType::UNKNOWN || _currentMode == OpenLogMode::ERROR_MODE) {
    return OpenLogStatus::ERROR;
  }

  if (_firmwareType == OpenLogFirmwareType::OPENLOG_STANDARD && _currentMode != OpenLogMode::LOGGING_MODE) {
    if (exitCommandMode() != OpenLogStatus::OK) {
      return OpenLogStatus::ERROR;
    }
  }

  _serial.print(data);
  return OpenLogStatus::OK;
}

OpenLogStatus OpenLogDriver::write(const char* data) {
  return write(String(data));
}

OpenLogStatus OpenLogDriver::write(byte data) {
  if (_firmwareType == OpenLogFirmwareType::UNKNOWN || _currentMode == OpenLogMode::ERROR_MODE) {
    return OpenLogStatus::ERROR;
  }
  if (_firmwareType == OpenLogFirmwareType::OPENLOG_STANDARD && _currentMode != OpenLogMode::LOGGING_MODE) {
    if (exitCommandMode() != OpenLogStatus::OK) {
      return OpenLogStatus::ERROR;
    }
  }
  _serial.write(data);
  return OpenLogStatus::OK;
}

OpenLogStatus OpenLogDriver::enterCommandMode() {
  if (_firmwareType != OpenLogFirmwareType::OPENLOG_STANDARD) {
    return OpenLogStatus::NOT_SUPPORTED;
  }
  if (_currentMode == OpenLogMode::COMMAND_MODE) {
    return OpenLogStatus::OK;
  }

  clearSerialBuffer();
  for (int i = 0; i < DEFAULT_ESCAPE_COUNT; ++i) {
    _serial.write(DEFAULT_ESCAPE_CHAR);
    delay(10);
  }
  _serial.print("\r");

  String response = readLine(1000);
  if (response.indexOf(">") != -1) {
    _currentMode = OpenLogMode::COMMAND_MODE;
    return OpenLogStatus::OK;
  }
  return OpenLogStatus::TIMEOUT;
}

OpenLogStatus OpenLogDriver::exitCommandMode() {
  if (_firmwareType != OpenLogFirmwareType::OPENLOG_STANDARD) {
    return OpenLogStatus::NOT_SUPPORTED;
  }
  if (_currentMode == OpenLogMode::LOGGING_MODE) {
    return OpenLogStatus::OK;
  }

  String response;
  OpenLogStatus status = sendCommand("new log.txt", response, 2000);
  if (status == OpenLogStatus::OK || response.indexOf("<") != -1) {
    _currentMode = OpenLogMode::LOGGING_MODE;
    return OpenLogStatus::OK;
  }
  return OpenLogStatus::ERROR;
}

OpenLogStatus OpenLogDriver::sendCommand(const String& command, String& response, unsigned long timeoutMs) {
  if (_firmwareType != OpenLogFirmwareType::OPENLOG_STANDARD) {
    return OpenLogStatus::NOT_SUPPORTED;
  }
  if (_currentMode != OpenLogMode::COMMAND_MODE) {
    if (enterCommandMode() != OpenLogStatus::OK) {
      return OpenLogStatus::ERROR;
    }
  }

  OpenLogStatus status = sendAndExpect(command, ">", response, timeoutMs);
  if (status == OpenLogStatus::OK) {
    int promptPos = response.indexOf(">");
    if (promptPos != -1) {
      response = response.substring(0, promptPos);
    }
    response.trim();
  }
  return status;
}

OpenLogStatus OpenLogDriver::queryFirmwareVersion(String& version) {
  if (_firmwareType != OpenLogFirmwareType::OPENLOG_STANDARD) {
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
  if (_firmwareType != OpenLogFirmwareType::OPENLOG_STANDARD) {
    return OpenLogStatus::NOT_SUPPORTED;
  }
  String response;
  OpenLogStatus status = sendCommand("baud " + String(baud), response, 1000);
  if (status == OpenLogStatus::OK) {
    _currentBaudRate = baud;
    return OpenLogStatus::OK;
  }
  return status;
}

OpenLogStatus OpenLogDriver::setMode(byte mode) {
  if (_firmwareType != OpenLogFirmwareType::OPENLOG_STANDARD) {
    return OpenLogStatus::NOT_SUPPORTED;
  }
  if (mode > 2) return OpenLogStatus::INVALID_PARAMETER;

  String response;
  return sendCommand("set " + String(mode), response, 1000);
}

OpenLogStatus OpenLogDriver::setVerbose(bool on) {
  if (_firmwareType != OpenLogFirmwareType::OPENLOG_STANDARD) {
    return OpenLogStatus::NOT_SUPPORTED;
  }
  String state = on ? "on" : "off";
  String response;
  return sendCommand("verb " + state, response, 1000);
}

OpenLogStatus OpenLogDriver::setEcho(bool on) {
  if (_firmwareType != OpenLogFirmwareType::OPENLOG_STANDARD) {
    return OpenLogStatus::NOT_SUPPORTED;
  }
  String state = on ? "on" : "off";
  String response;
  return sendCommand("echo " + state, response, 1000);
}

OpenLogStatus OpenLogDriver::syncData() {
  if (_firmwareType == OpenLogFirmwareType::UNKNOWN || _currentMode == OpenLogMode::ERROR_MODE) {
    return OpenLogStatus::ERROR;
  }

  if (_firmwareType == OpenLogFirmwareType::OPENLOG_STANDARD) {
    OpenLogMode previousMode = _currentMode;
    if (enterCommandMode() != OpenLogStatus::OK) {
      return OpenLogStatus::ERROR;
    }
    String response;
    OpenLogStatus status = sendCommand("sync", response, 1000);
    if (previousMode == OpenLogMode::LOGGING_MODE) {
      exitCommandMode();
    }
    return status;
  } else {
    _serial.print("sync\r");
    delay(500);
    clearSerialBuffer();
    return OpenLogStatus::OK;
  }
}

bool OpenLogDriver::isSDCardOK() {
  if (_currentMode == OpenLogMode::ERROR_MODE) {
    return false;
  }
  if (_firmwareType == OpenLogFirmwareType::OPENLOG_STANDARD) {
    String response;
    OpenLogStatus status = sendCommand("disk", response, 2000);
    if (status == OpenLogStatus::OK && response.indexOf("Card Size:") != -1) {
      return true;
    }
    return false;
  }
  return true;
}

OpenLogStatus OpenLogDriver::getConfig(OpenLogConfig& config) {
  if (_firmwareType != OpenLogFirmwareType::OPENLOG_STANDARD) {
    return OpenLogStatus::NOT_SUPPORTED;
  }
  String response;
  OpenLogStatus status = sendCommand("?", response, 1000);
  if (status == OpenLogStatus::OK) {
    config.baudRate = _currentBaudRate;
    config.escapeChar = DEFAULT_ESCAPE_CHAR;
    config.escapeCharCount = DEFAULT_ESCAPE_COUNT;
    config.mode = 0;
    config.verbose = 0;
    config.echo = 0;
    config.ignoreRX = 0;
    return OpenLogStatus::OK;
  }
  return status;
}

OpenLogStatus OpenLogDriver::setConfig(const OpenLogConfig& config) {
  if (_firmwareType != OpenLogFirmwareType::OPENLOG_STANDARD) {
    return OpenLogStatus::NOT_SUPPORTED;
  }
  OpenLogStatus status;
  String response;

  status = sendCommand("baud " + String(config.baudRate), response, 1000);
  if (status != OpenLogStatus::OK) return status;

  status = sendCommand("escape " + String(config.escapeChar), response, 1000);
  if (status != OpenLogStatus::OK) return status;

  status = sendCommand("esc# " + String(config.escapeCharCount), response, 1000);
  if (status != OpenLogStatus::OK) return status;

  status = sendCommand("set " + String(config.mode), response, 1000);
  if (status != OpenLogStatus::OK) return status;

  status = sendCommand("verb " + String(config.verbose), response, 1000);
  if (status != OpenLogStatus::OK) return status;

  status = sendCommand("echo " + String(config.echo), response, 1000);
  if (status != OpenLogStatus::OK) return status;

  status = sendCommand("ignoreRX " + String(config.ignoreRX), response, 1000);
  if (status != OpenLogStatus::OK) return status;

  _currentConfig = config;
  return OpenLogStatus::OK;
}

void OpenLogDriver::setVoltageMonitoring(int adcPin, float voltageDividerRatio, float brownoutThreshold) {
  _voltageAdcPin = adcPin;
  _voltageDividerRatio = voltageDividerRatio;
  _brownoutThreshold = brownoutThreshold;
  _voltageMonitoringEnabled = true;
}

OpenLogStatus OpenLogDriver::checkPowerStatus() {
  if (!_voltageMonitoringEnabled || _voltageAdcPin == -1) {
    return OpenLogStatus::NOT_SUPPORTED;
  }

  int adcRaw = analogRead(_voltageAdcPin);
  float voltage = (float)adcRaw / 4095.0 * 3.3 * _voltageDividerRatio;

  if (voltage < _brownoutThreshold) {
    return OpenLogStatus::BROWNOUT_IMMINENT;
  } else if (voltage < (_brownoutThreshold * 1.1)) {
    return OpenLogStatus::LOW_VOLTAGE_WARNING;
  }
  return OpenLogStatus::OK;
}
