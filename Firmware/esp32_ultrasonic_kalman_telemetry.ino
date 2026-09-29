#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>

// wifi ap and network settings
const char* ssid = "Project_78_Drone";
const char* password = "12345678"; 

WiFiUDP udp;
const int udpPort = 4210;
IPAddress broadcastIp(192, 168, 4, 255); 

// serial pins for lidar connection
HardwareSerial lidarSerial(2); 
const int RXD2 = 33;
const int TXD2 = 25;
float current_lidar_cm = 0.0;

// mutex vars to share data between core 0 and core 1
SemaphoreHandle_t dataMutex;
float shared_raw_dist = 0.0;
float shared_kalman_dist = 0.0;
float shared_lidar_dist = 0.0; 

// ultrasonic pins and math helper
const int TRIG_PIN = 32;
const int ECHO_PIN = 35;
const float CM_CONVERSION_FACTOR = 57.7; 

// look up tables for dynamic bias fix
const int NUM_CALIBRATION_POINTS = 4;
const float CALIB_DISTANCES[NUM_CALIBRATION_POINTS] = {25.0, 50.0, 100.0, 200.0};
const float CALIB_BIASES[NUM_CALIBRATION_POINTS] = {-1.71,  -0.33,  -2.04,  -0.19};

// linear interpolation for sonar bias compensation
float getDynamicBias(float rawDist) {
  if (rawDist <= CALIB_DISTANCES[0]) return CALIB_BIASES[0];
  if (rawDist >= CALIB_DISTANCES[NUM_CALIBRATION_POINTS - 1]) return CALIB_BIASES[NUM_CALIBRATION_POINTS - 1];
  
  for (int i = 0; i < NUM_CALIBRATION_POINTS - 1; i++) {
    if (rawDist >= CALIB_DISTANCES[i] && rawDist < CALIB_DISTANCES[i+1]) {
      return CALIB_BIASES[i] + ((rawDist - CALIB_DISTANCES[i]) / (CALIB_DISTANCES[i+1] - CALIB_DISTANCES[i])) * (CALIB_BIASES[i+1] - CALIB_BIASES[i]);
    }
  }
  return 0.0; 
}

// linear kalman filter class (constant velocity matrix math)
class KalmanFilterCV {
  private:
    float x_dist; float x_vel;  
    float P00, P01, P10, P11; 
    float R;
    float Q11, Q12, Q21, Q22; 
  public:
    KalmanFilterCV(float initial_distance) {
      x_dist = initial_distance; x_vel = 0.0; 
      P00 = 10.0; P01 = 0.0; P10 = 0.0; P11 = 10.0;
      R = 0.000396; // tuned measurement covariance
    }
    
    void predict(float dt) {
      float sigma_a_sq = 3.36; // process noise variance
      float dt2 = dt * dt;
      float dt3 = dt2 * dt;
      float dt4 = dt3 * dt;
      
      Q11 = sigma_a_sq * (0.25 * dt4);
      Q12 = sigma_a_sq * (0.5 * dt3);
      Q21 = Q12;
      Q22 = sigma_a_sq * dt2;
      
      x_dist = x_dist + x_vel * dt;
      
      P00 = P00 + dt * (P10 + P01) + dt2 * P11 + Q11;
      P01 = P01 + dt * P11 + Q12; 
      P10 = P10 + dt * P11 + Q21; 
      P11 = P11 + Q22;
    }
    
    void update(float z_meas) {
      float S = P00 + R; float K0 = P00 / S; float K1 = P10 / S; 
      float y = z_meas - x_dist;
      x_dist = x_dist + K0 * y; x_vel = x_vel + K1 * y;
      float P00_temp = P00; float P01_temp = P01;
      P00 = (1 - K0) * P00_temp; P01 = (1 - K0) * P01_temp;
      P10 = -K1 * P00_temp + P10; P11 = -K1 * P01_temp + P11;
    }
    float getDistance() { return x_dist; }
};

KalmanFilterCV kf(0.0);

// isr variables for non blocking echo pulse timing
volatile unsigned long echoStart = 0;
volatile unsigned long echoEnd = 0;
volatile bool newMeasurement = false;
unsigned long lastUpdateTime = 0;
unsigned long lastTime = 0;

// hardware interrupt function on echo pin state change
void IRAM_ATTR echoISR() {
  if (digitalRead(ECHO_PIN) == HIGH) {
    echoStart = micros();
  } else {
    echoEnd = micros();
    newMeasurement = true;
  }
}

// ==============================================================================
// CORE 0: Telemetry Task (WiFi & UDP)
// ==============================================================================
void WiFiTask(void *pvParameters) {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(ssid, password, 6, 0, 1); 
  udp.begin(udpPort);
  char packetBuffer[60]; 

  for(;;) {
    // secure shared variables reading
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    float currentRaw = shared_raw_dist;
    float currentKalman = shared_kalman_dist;
    float currentLidar = shared_lidar_dist; 
    xSemaphoreGive(dataMutex);
    
    // pack string payload with tabs and broadcast
    sprintf(packetBuffer, "%.2f\t%.2f\t%.2f", currentRaw, currentKalman, currentLidar);
    udp.beginPacket(broadcastIp, udpPort);
    udp.print(packetBuffer);
    udp.endPacket();
    vTaskDelay(33 / portTICK_PERIOD_MS); // maintain 30hz transmission loop
  }
}

// ==============================================================================
// SETUP
// ==============================================================================
void setup() {
  Serial.begin(115200);
  lidarSerial.begin(115200, SERIAL_8N1, RXD2, TXD2); // tf-luna default uart configuration
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  dataMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 10000, NULL, 1, NULL, 0); // pin telemetry to core 0
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);
  lastTime = millis();
  lastUpdateTime = millis();
  pinMode(26, OUTPUT); // physical warning led output
}

// ==============================================================================
// CORE 1: Main Loop (Sensors & Math)
// ==============================================================================
void loop() {
  static TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 33 / portTICK_PERIOD_MS; // target 30hz execution

  // parse 9 byte data packets from lidar serial buffer
  while (lidarSerial.available() >= 9) {
    if (lidarSerial.read() == 0x59) {
      if (lidarSerial.peek() == 0x59) {
        uint8_t frame[9]; frame[0] = 0x59; frame[1] = lidarSerial.read(); 
        uint16_t checksum = frame[0] + frame[1];
        for(int i = 2; i < 9; i++) {
          frame[i] = lidarSerial.read();
          if (i < 8) checksum += frame[i];
        }
        if (frame[8] == (checksum & 0xFF)) {
          current_lidar_cm = frame[2] + (frame[3] << 8); // merge bytes for true distance
        }
      }
    }
  }

  // prediction cycle tracking
  unsigned long currentTime = millis();
  float dt_predict = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  kf.predict(dt_predict);

  unsigned long currentPulseWidth = 0; 
  float correctedDistCm = 0.0;
  bool isMeasurementValid = false;

  // handle completed sonar pulse readings
  if (newMeasurement) {
    currentPulseWidth = echoEnd - echoStart;
    newMeasurement = false; 

    // software threshold timeout protection filter
    if (currentPulseWidth < 30000 && currentPulseWidth > 100) { 
      float rawDistCm = currentPulseWidth / CM_CONVERSION_FACTOR; 
      float currentBias = getDynamicBias(rawDistCm);
      correctedDistCm = rawDistCm - currentBias; // drop fixed bias error
      
      float dt_update = (currentTime - lastUpdateTime) / 1000.0;
      lastUpdateTime = currentTime; 
      
      kf.update(correctedDistCm); // fix kalman matrices state
      isMeasurementValid = true;
    }
  }

  float kalmanDistCm = kf.getDistance();

  // physical led warning logic (trigger under 1 meter safety mark)
  if (kalmanDistCm < 100.0) {
    digitalWrite(26, HIGH);
  } else {
    digitalWrite(26, LOW);
  }
  
  // write state data to global vars using data mutex
  if(xSemaphoreTake(dataMutex, 0) == pdTRUE) {
    shared_raw_dist = isMeasurementValid ? correctedDistCm : ((float)currentPulseWidth / CM_CONVERSION_FACTOR); 
    shared_kalman_dist = kalmanDistCm; 
    shared_lidar_dist = current_lidar_cm;
    xSemaphoreGive(dataMutex);
  }

  // launch new pulse only if echo pin cleared to low state
  if (digitalRead(ECHO_PIN) == LOW) {
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  }
  
  vTaskDelayUntil(&xLastWakeTime, xFrequency); // sleep until next execution frame
}
