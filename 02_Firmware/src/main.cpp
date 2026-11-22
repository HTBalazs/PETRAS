#include <Arduino.h>
#include <esp_sleep.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <ICM42670P.h>
#include <vector>
#include <queue>
#include <esp_bt.h>
#include <SimpleFTPServer.h>
#include <cstdarg>

class Petras {
private:
  IPAddress local_IP{192, 168, 1, 1};
  IPAddress gateway{192, 168, 1, 1};
  IPAddress subnet{255, 255, 255, 0};
  const char* hostname = "PETRAS";
  const char* ssid = "PETRAS";
  const char* password = "smartpebble";
  static constexpr uint8_t cpuFreqActiveMHz = 240; // Used while sensor timing precision matters (measurement, live test, calibration)
  static constexpr uint8_t cpuFreqIdleMHz = 80; // Lowest frequency that keeps the WiFi AP stable
  ICM42670 IMU{Wire,1};
  File eventLog;
  String eventLogFileName = "/eventLog.txt";
  uint16_t filewriteInterval = 1000; // The time between two consecutive readings/file write operations in microseconds
  uint32_t flushInterval = 1000000; // The time between two flushes in microseconds
  uint8_t accelRange = 16; // Given in g; +/-; so the total measurement range is from -accelRange*9.81 [m/s^2] to +accelRange*9.81 [m/s^2]; can be set to 2, 4, 8, 16
  uint16_t gyroRange = 2000; // Given in degrees per second (dps); +/-; so the total measurement range is from -gyroRange [°/s] to +gyroRange ]°/s]; can be set to 250, 500, 1000, 2000
  float gScale = 9.81; // Because the sensor output is given in g by default
  int int16L = -32768; // Lower limit of the 16-bit signed integer for the conversion from raw sensor data to meaningful physical quantity
  int int16H = 32767; // Upper limit of the 16-bit signed integer for the conversion from raw sensor data to meaningful physical quantity
  int ODR = 1600; // The sampling frequency of the sensor (ODR = output data rate)
  WiFiServer server{80};
  WiFiClient client;
  FtpServer ftp;
  bool ftpServerOn = false;
  const char* ftp_user = "admin";
  const char* ftp_pass = "admin";
  bool configMode = false;
  String configFileName = "/config.txt";
  std::queue<String> inputList;
  bool liveTestRunning = false;
  uint32_t liveTestStartTime = 0;
  float accelOffsetX = 0.0f;
  float accelOffsetY = 0.0f;
  float accelOffsetZ = 0.0f;
  float gyroOffsetX = 0.0f;
  float gyroOffsetY = 0.0f;
  float gyroOffsetZ = 0.0f;
  float gyroScaleX = 1.0f;
  float gyroScaleY = 1.0f;
  float gyroScaleZ = 1.0f;
  String calibrationFileName = "/calibration.txt";
private:
  template<typename T>
  void logPrint(T const& value) {
    eventLog.print(value);
    if (client && client.connected() && !configMode) {
      client.print(value);
    }
  }
  template<typename T>
  void logPrintln(T const& value) {
    eventLog.println(value);
    if (client && client.connected() && !configMode) {
      client.println(value);
    }
  }
  void logPrintln() {
    eventLog.println();
    if (client && client.connected() && !configMode) {
      client.println();
    }
  }
  void logPrintf(const char* format, ...) {
    char buf[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    char* buffer = buf;
    bool allocated = false;
    if (len >= (int)sizeof(buf)) {
      buffer = new char[len + 1];
      allocated = true;
      va_start(args, format);
      vsnprintf(buffer, len + 1, format, args);
      va_end(args);
    }
    logPrint(buffer);
    if (allocated) {
      delete[] buffer;
    }
  }
  void useWiFi(bool turnOn) {
    if (!turnOn && WiFi.getMode() != WIFI_OFF) {
      logPrintln("Turn off WiFi...");
      server.stop();
      WiFi.softAPdisconnect(true); 
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
    } else if (turnOn && WiFi.getMode() == WIFI_OFF) {
      WiFi.mode(WIFI_AP);
      WiFi.softAPConfig(local_IP, gateway, subnet);
      logPrintln("Configuring WiFi hotspot...");
      if (WiFi.softAP(ssid, password, 1, 0, 2)) {
        logPrintln("WiFi hotspot configured successfully.");
        logPrint("IP Address: ");
        logPrintln(WiFi.softAPIP());
      } else {
        logPrintln("Error occured during hotspot configuration.");
      }
      eventLog.flush();
      server.begin();
    }
  }
  void commandManager(String input, String& command, std::vector<String>& parameters) {
    input.trim();
    int separatorIndex = input.indexOf(':');
    if (separatorIndex != -1) {
      command = input.substring(0, separatorIndex);
      parameters = splitParameters(input.substring(separatorIndex+1));
    } else {
      command = input;
    }
  }
  std::vector<String> splitParameters(String params) {
    std::vector<String> sequence;
    int startPos = 0;
    int commaIndex = params.indexOf(',');
    while (commaIndex != -1) {
      String darab = params.substring(startPos, commaIndex);
      darab.trim();
      sequence.push_back(darab);
      startPos = commaIndex + 1;
      commaIndex = params.indexOf(',', startPos);
    }
    String lastOne = params.substring(startPos);
    lastOne.trim();
    if (lastOne.length() > 0) {
      sequence.push_back(lastOne);
    }
    return sequence;
  }
  bool isConfigMode() {
    inputList = std::queue<String>{};
    File configFile = SD.open(configFileName, FILE_READ);
    if (configFile) {
      while (configFile.available()) {
        String line = configFile.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
          inputList.push(line);
        }
      }
      configFile.close();
      return true;
    }
    return false;
  }
  void liveTestProcess() {
    static uint32_t lastSampleMillis = 0;
    static uint32_t lastSendMillis   = 0;
    static String  samples          = "";
    static uint8_t  sampleCount      = 0;
    const uint32_t SAMPLE_INTERVAL = 10;   // ms – mintavételezés
    const uint32_t SEND_INTERVAL   = 50;   // ms – küldés
    const uint8_t  BATCH_SIZE      = 5;    // 50ms / 10ms
    uint32_t now = millis();
    if (now - lastSampleMillis >= SAMPLE_INTERVAL) {
      lastSampleMillis = now;
      String sensorData = readSensorData(liveTestStartTime);
      samples += (sampleCount == 0 ? "" : ";") + sensorData;
      sampleCount++;
    }
    if (now - lastSendMillis >= SEND_INTERVAL && sampleCount > 0) {
      lastSendMillis = now;
      client.printf("DATA:%s\r\n", samples.c_str());
      samples     = "";
      sampleCount = 0;
    }
  }
  void saveCalibration() {
    File calibFile = SD.open(calibrationFileName.c_str(), FILE_WRITE);
    if (!calibFile) return;
    calibFile.printf("accelOffsetX:%.6f\n", accelOffsetX);
    calibFile.printf("accelOffsetY:%.6f\n", accelOffsetY);
    calibFile.printf("accelOffsetZ:%.6f\n", accelOffsetZ);
    calibFile.printf("gyroOffsetX:%.6f\n", gyroOffsetX);
    calibFile.printf("gyroOffsetY:%.6f\n", gyroOffsetY);
    calibFile.printf("gyroOffsetZ:%.6f\n", gyroOffsetZ);
    calibFile.printf("gyroScaleX:%.6f\n", gyroScaleX);
    calibFile.printf("gyroScaleY:%.6f\n", gyroScaleY);
    calibFile.printf("gyroScaleZ:%.6f\n", gyroScaleZ);
    calibFile.close();
    logPrintln("Calibration data saved to microSD.");
    eventLog.flush();
  }
  void loadCalibration() {
    File calibFile = SD.open(calibrationFileName.c_str(), FILE_READ);
    if (!calibFile) return;
    while (calibFile.available()) {
        String line = calibFile.readStringUntil('\n');
        line.trim();
        int sep = line.indexOf(':');
        if (sep == -1) continue;
        String key = line.substring(0, sep);
        float val = line.substring(sep + 1).toFloat();
        if (key == "accelOffsetX") accelOffsetX = val;
        else if (key == "accelOffsetY") accelOffsetY = val;
        else if (key == "accelOffsetZ") accelOffsetZ = val;
        else if (key == "gyroOffsetX")  gyroOffsetX = val;
        else if (key == "gyroOffsetY")  gyroOffsetY = val;
        else if (key == "gyroOffsetZ")  gyroOffsetZ = val;
        else if (key == "gyroScaleX")   gyroScaleX = val;
        else if (key == "gyroScaleY")   gyroScaleY = val;
        else if (key == "gyroScaleZ")   gyroScaleZ = val;
    }
    calibFile.close();
    logPrintln("Calibration data loaded from microSD.");
    eventLog.flush();
  }
public:
  Petras() {}
  void initialize() {
    // Turn off BT to save some power
    btStop();
    esp_bt_controller_disable();
    esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
    // Initializing the ICM42670
    Wire.begin(33, 34);
    delay(1000);
    int ret = IMU.begin();
    IMU.startAccel(ODR,accelRange);
    IMU.startGyro(ODR,gyroRange);
    delay(1000);
    // Configure the microSD card
    SD.begin();
    purgeData();
    eventLog = SD.open(eventLogFileName, FILE_WRITE);
    loadCalibration();
    logPrintln("The microSD card initialized successfully.");
    eventLog.flush();
    configMode = isConfigMode();
    if (!ret) { logPrintln("ICM-42670-P IMU initialized"); } else { logPrintln("ICM-42670-P IMU initialization failed."); }
    // Configure WiFi hotspot
    useWiFi(!configMode);
    logPrintln("\n#### The PETRAS setup procedure is successfully completed. ####\n");
    logPrintf("PETRAS runs in %s mode.\n", configMode?"standalone":"remote");
    eventLog.flush();
    if(WiFi.getMode() != WIFI_OFF) {
      ArduinoOTA.setHostname("PETRAS");
      ArduinoOTA.begin();
    }
    setCpuFrequencyMhz(cpuFreqIdleMHz);
  }
  void purgeData(bool const keepEventLog = false) {
    File root = SD.open("/");
    if (!root) return;
    File file = root.openNextFile();
    while (file) {
      String fileName = file.path();
      if (!file.isDirectory()) {
        if (!(fileName == eventLogFileName && keepEventLog) && fileName != configFileName && fileName != "/calibration.txt") {
          file.close();
          SD.remove(fileName);
        } else {
          file.close();
        }
      } else {
        file.close();
      }
      file = root.openNextFile();
    }
    root.close();
    delay(500);
  }
  String readSensorData(uint32_t const& startTime) {
    inv_imu_sensor_event_t imu_event;
    IMU.getDataFromRegisters(imu_event);
    float elapsedTime = (float)(micros()-startTime)/1000000.0;
    float elapsedTime2 = (float)imu_event.timestamp_fsync;
    float realAccelX = (float)imu_event.accel[0]/int16L*accelRange*gScale - accelOffsetX;
    float realAccelY = (float)imu_event.accel[1]/int16L*accelRange*gScale - accelOffsetY;
    float realAccelZ = (float)imu_event.accel[2]/int16L*accelRange*gScale - accelOffsetZ;
    float realGyroX = ((float)imu_event.gyro[0]/int16L*gyroRange - gyroOffsetX) * gyroScaleX;
    float realGyroY = ((float)imu_event.gyro[1]/int16L*gyroRange - gyroOffsetY) * gyroScaleY;
    float realGyroZ = ((float)imu_event.gyro[2]/int16L*gyroRange - gyroOffsetZ) * gyroScaleZ;
    float realTemp = (float)(imu_event.temperature/128)+25; // Conversion from the official datasheet
    char buf[128];
    snprintf(buf, sizeof(buf), "%1.6f, %1.6f, %1.6f, %1.6f, %1.6f, %1.6f, %1.6f", elapsedTime, realAccelX, realAccelY, realAccelZ, realGyroX, realGyroY, realGyroZ);
    return String{buf};
  }
  void startMeasurement(String const fileName, double const duration, bool keepWiFiOn) {
    setCpuFrequencyMhz(cpuFreqActiveMHz);
    logPrintf("Measurement started. File name: %s, Duration: %1.2f s\n", fileName.c_str(), duration);
    eventLog.flush();
    uint32_t last_filewrite = 0;
    uint32_t last_flush = 0;
    uint32_t startTime = micros();
    File measData = SD.open(fileName, FILE_WRITE);
    measData.println("Estimated Time [s], Acceleration X, Y, Z [m/s^2], Angular Velocity X, Y, Z [deg/s]");
    if (!keepWiFiOn && !configMode) {
      useWiFi(false);
    }
    while (micros() - startTime < duration * 1e6) {
      // Avoid watchdog shutting down the measurement
      yield();
      if (micros() - last_filewrite > filewriteInterval) {
        String sensorData = readSensorData(startTime);
        last_filewrite = micros();
        measData.println(sensorData.c_str());
      }
      // Flushing
      if (micros() - last_flush > flushInterval) {
        measData.flush();
        last_flush = micros();
      }
    }
    measData.close();
    setCpuFrequencyMhz(cpuFreqIdleMHz);
    if (!keepWiFiOn && !configMode) {
      useWiFi(true);
    }
  }
 
  void loop() {
    if (WiFi.getMode() == WIFI_OFF && !configMode) {
      useWiFi(true);
    }
    if(WiFi.getMode() != WIFI_OFF) {
      ArduinoOTA.handle();
    }
    if (ftpServerOn) {
      ftp.handleFTP();
      delay(100);
    }
    if ((!client || !client.connected()) && WiFi.getMode() != WIFI_OFF) {
      WiFiClient newClient = server.accept();
      if (newClient) {
        client = newClient;
        client.setNoDelay(true);
        IPAddress ip = client.remoteIP();
        logPrint("A new client is connected with the IP address: ");
        logPrintln(ip);
        eventLog.flush();
      }
    }
    if (liveTestRunning) {
      liveTestProcess();
    }
    String input = "";
    if (configMode) {
      if (!inputList.empty()) {
        input = inputList.front();
        inputList.pop();
        if (inputList.empty()) {
          configMode = false;
        }
      }
    } else if (client && client.connected() && client.available()>0) {
      input = client.readStringUntil('\n');
    }
    if (!input.isEmpty()) {
      String command = "";
      static std::vector<String> parameters;
      commandManager(input, command, parameters);
      if (command == "RESET") {
        ESP.restart();
      } else if (command == "START_MEASUREMENT") {
        startMeasurement(parameters[0], parameters[1].toInt(), parameters[2].toInt()?true:false);
        client.println("DONE:START_MEASUREMENT");
      } else if (command == "PURGE_DATA") {
        purgeData(true);
        logPrintln("Data purged.");
        eventLog.flush();
      } else if (command == "START_LIVE_TEST" && !configMode && !liveTestRunning) {
        setCpuFrequencyMhz(cpuFreqActiveMHz);
        liveTestRunning = true;
        liveTestStartTime = micros();
        logPrintln("Live test started...");
        eventLog.flush();
      } else if (command == "STOP_LIVE_TEST" && !configMode && liveTestRunning) {
        liveTestRunning = false;
        setCpuFrequencyMhz(cpuFreqIdleMHz);
        logPrintln("Live test finished...");
        eventLog.flush();
      } else if (command == "START_FTP_SERVER" && !configMode) {
        ftp.setLocalIp(WiFi.softAPIP());
        ftp.begin(ftp_user, ftp_pass);
        ftp.setLocalIp(WiFi.softAPIP());
        ftpServerOn = true;
        logPrintln("FTP Server is started.");
        eventLog.flush();
      } else if (command == "STOP_FTP_SERVER" && !configMode) {
        ftp.end();
        ftpServerOn = false;
        logPrintln("FTP Server has been shut down.");
        eventLog.flush();
      } else if (command == "WAIT") {
        int waitSeconds = parameters[0].toInt();
        logPrintf("Wait %d seconds...\n", waitSeconds);
        eventLog.flush();
        delay(waitSeconds*1000);
      } else if (command == "SLEEP") {
        int sleepSeconds = parameters[0].toInt();
        esp_sleep_enable_timer_wakeup(sleepSeconds*1000000);
        logPrintf("Sleep %d seconds...\n", sleepSeconds);
        eventLog.flush();
        useWiFi(false);
        esp_light_sleep_start();
        logPrintf("Woke up...\n");
        eventLog.flush();
        useWiFi(!configMode);
      } else if (command == "START_ACCEL_OFFSET") {
        int duration = parameters[0].toInt();
        logPrintf("Start Acceleration offset calibration for %d seconds...\n", duration);
        float sumX = 0, sumY = 0, sumZ = 0;
        int count = 0;
        setCpuFrequencyMhz(cpuFreqActiveMHz);
        uint32_t startTime = micros();
        while (micros() - startTime < (uint32_t)duration * 1000000UL) {
            yield();
            inv_imu_sensor_event_t imu_event;
            IMU.getDataFromRegisters(imu_event);
            sumX += (float)imu_event.accel[0] / int16L * accelRange * gScale;
            sumY += (float)imu_event.accel[1] / int16L * accelRange * gScale;
            sumZ += (float)imu_event.accel[2] / int16L * accelRange * gScale;
            count++;
            delay(1);
        }
        setCpuFrequencyMhz(cpuFreqIdleMHz);
        accelOffsetX = (count > 0) ? sumX / count : 0.0f;
        accelOffsetY = (count > 0) ? sumY / count : 0.0f;
        accelOffsetZ = (count > 0) ? sumZ / count : 0.0f-gScale;
        logPrintf("Calibration is done, results: %1.6f, %1.6f, %1.6f\n", accelOffsetX, accelOffsetY, accelOffsetZ);
        eventLog.flush();
        saveCalibration();
        client.println("DONE:START_ACCEL_OFFSET");
      } else if (command == "START_GYRO_OFFSET") {
        int duration = parameters[0].toInt();
        logPrintf("Start Angular velocity offset calibration for %d seconds...\n", duration);
        float sumX = 0, sumY = 0, sumZ = 0;
        int count = 0;
        setCpuFrequencyMhz(cpuFreqActiveMHz);
        uint32_t startTime = micros();
        while (micros() - startTime < (uint32_t)duration * 1000000UL) {
            yield();
            inv_imu_sensor_event_t imu_event;
            IMU.getDataFromRegisters(imu_event);
            sumX += (float)imu_event.gyro[0]/int16L*gyroRange;
            sumY += (float)imu_event.gyro[1]/int16L*gyroRange;
            sumZ += (float)imu_event.gyro[2]/int16L*gyroRange;
            count++;
            delay(1);
        }
        setCpuFrequencyMhz(cpuFreqIdleMHz);
        gyroOffsetX = (count > 0) ? sumX / count : 0.0f;
        gyroOffsetY = (count > 0) ? sumY / count : 0.0f;
        gyroOffsetZ = (count > 0) ? sumZ / count : 0.0f;
        logPrintf("Calibration is done, results: %1.6f, %1.6f, %1.6f\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
        eventLog.flush();
        saveCalibration();
        client.println("DONE:START_GYRO_OFFSET");
      } else if (command == "START_GYRO_SCALE") {
          String axis = parameters[0];
          int duration = parameters[1].toInt();
          logPrintf("Start Angular velocity %s scale calibration for %d seconds...\n", axis, duration);
          float refVel = parameters[2].toFloat();
          float sum = 0;
          int count = 0;
          setCpuFrequencyMhz(cpuFreqActiveMHz);
          uint32_t startTime = micros();
          while (micros() - startTime < (uint32_t)duration * 1000000UL) {
              yield();
              inv_imu_sensor_event_t imu_event;
              IMU.getDataFromRegisters(imu_event);
              if (axis == "X")      sum += (float)imu_event.gyro[0]/int16L*gyroRange - gyroOffsetX;
              else if (axis == "Y") sum += (float)imu_event.gyro[1]/int16L*gyroRange - gyroOffsetY;
              else if (axis == "Z") sum += (float)imu_event.gyro[2]/int16L*gyroRange - gyroOffsetZ;
              count++;
              delay(1);
          }
          setCpuFrequencyMhz(cpuFreqIdleMHz);
          float measuredAvg = (count > 0) ? sum / count : 1.0f;
          float scale = (measuredAvg != 0.0f) ? refVel / measuredAvg : 1.0f;
          if (axis == "X")      gyroScaleX = scale;
          else if (axis == "Y") gyroScaleY = scale;
          else if (axis == "Z") gyroScaleZ = scale;
          logPrintf("Calibration is done, result: %1.6f\n", scale);
          eventLog.flush();
          saveCalibration();
          client.println("DONE:START_GYRO_SCALE");
      }
    }
    if (!liveTestRunning && !ftpServerOn) {
      delay(100);
    }
  }
};

Petras petras;

void setup() {
  petras.initialize();
}

void loop() {
  petras.loop();
}