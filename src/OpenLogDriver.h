#ifndef OPENLOG_DRIVER_H
#define OPENLOG_DRIVER_H

#include <Arduino.h>
#include <HardwareSerial.h>

enum class OpenLogFirmwareType {
    UNKNOWN,
    OPENLOG_STANDARD,
    OPENLOG_LIGHT,
    OPENLOG_MINIMAL
};

enum class OpenLogMode {
    UNKNOWN,
    LOGGING_MODE,
    COMMAND_MODE,
    ERROR_MODE
};

enum class OpenLogStatus {
    OK,
    ERROR,
    TIMEOUT,
    NOT_SUPPORTED,
    INVALID_PARAMETER,
    SD_CARD_ERROR,
    FIRMWARE_MISMATCH,
    LOW_VOLTAGE_WARNING,
    BROWNOUT_IMMINENT
};

struct OpenLogConfig {
    long baudRate;
    byte escapeChar;
    byte escapeCharCount;
    byte mode;
    byte verbose;
    byte echo;
    byte ignoreRX;
};

class OpenLogDriver {
public:
    enum class InitState {
        NOT_STARTED,
        RESETTING,
        PROBING_BAUD,
        DETECTING_FIRMWARE,
        COMPLETE,
        ERROR
    };

    // Konstruktor mit RX/TX Pins
    OpenLogDriver(HardwareSerial& serialPort, int rxPin, int txPin, int dtrGrnPin, 
                 OpenLogFirmwareType expectedFirmware = OpenLogFirmwareType::UNKNOWN);
    
    InitState beginAsync();
    InitState getInitState() const { return _initState; }
    
    OpenLogFirmwareType getFirmwareType() const { return _firmwareType; }
    OpenLogMode getOpenLogMode() const { return _currentMode; }
    long getCurrentBaudRate() const { return _currentBaudRate; }
    
    OpenLogStatus write(const String& data);
    OpenLogStatus write(const char* data);
    OpenLogStatus write(byte data);
    
    OpenLogStatus enterCommandMode();
    OpenLogStatus exitCommandMode();
    OpenLogStatus sendCommand(const String& command, String& response, unsigned long timeoutMs = 1000);
    OpenLogStatus queryFirmwareVersion(String& version);
    OpenLogStatus setBaudRate(long baud);
    OpenLogStatus setMode(byte mode);
    OpenLogStatus setVerbose(bool on);
    OpenLogStatus setEcho(bool on);
    OpenLogStatus syncData();
    
    void resetOpenLogHardware();
    bool isSDCardOK();
    OpenLogStatus getConfig(OpenLogConfig& config);
    OpenLogStatus setConfig(const OpenLogConfig& config);
    
    void setVoltageMonitoring(int adcPin, float voltageDividerRatio, float brownoutThreshold);
    OpenLogStatus checkPowerStatus();

private:
    HardwareSerial& _serial;
    int _rxPin;
    int _txPin;
    int _dtrGrnPin;
    OpenLogFirmwareType _firmwareType = OpenLogFirmwareType::UNKNOWN;
    OpenLogMode _currentMode = OpenLogMode::UNKNOWN;
    long _currentBaudRate = 0;
    OpenLogConfig _currentConfig;
    OpenLogFirmwareType _expectedFirmware;
    
    InitState _initState = InitState::NOT_STARTED;
    unsigned long _initStartTime = 0;
    int _currentBaudIndex = 0;
    unsigned long _lastProbeTime = 0;
    
    int _voltageAdcPin = -1;
    float _voltageDividerRatio = 1.0;
    float _brownoutThreshold = 0.0;
    bool _voltageMonitoringEnabled = false;

    String readLine(unsigned long timeoutMs);
    OpenLogStatus sendAndExpect(const String& data, const String& expectedPrompt, 
                               String& response, unsigned long timeoutMs);
    OpenLogMode parseStartupPrompt(const String& prompt);
    void clearSerialBuffer();
};

#endif
