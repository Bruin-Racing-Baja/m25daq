#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_GPS.h>
#include <USBHost_t36.h>
#include <TimeLib.h>
#include <SD.h>
#include <SPI.h>
// OLED setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3D
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// IMU setup
#define IMU_SCL 24
#define IMU_SDA 25
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// GPS USB setup
USBHost myusb;
USBSerial_BigBuffer gpsSerial(myusb);
#define UPDATE_INTERVAL 100
uint32_t lastUpdate = 0;

#define LED1_PIN 4
#define LED2_PIN 9
#define LED3_PIN 10

time_t get_teensy3_time() { return Teensy3Clock.get(); }
String lastLine = "";
bool gpsFix = false;
bool sdFail = false;
bool logFail = false;
bool imuFail = false;
int hall;
int shock_pot;
int shock_pot2;
float roll = 0;
float pitch = 0;
float yaw = 0;
float lateralAccel = 0;
float forwardAccel = 0;
float heaveAccel = 0;
float ax = 0;
float ay = 0;
float az = 0;
float gyx = 0;
float rw = 0;
float rx = 0;
float ry = 0;
float rz = 0;
float pitchRate = 0;
float rollRate = 0;
float yawRate = 0;
float gpsLat = 0.0;
float gpsLon = 0.0;
char log_name[32];



File logFile;
// Convert NMEA lat/lon to decimal degrees
float convertToDecimal(float nmeaCoord, String direction) {
    int degrees = int(nmeaCoord / 100);
    float minutes = nmeaCoord - (degrees * 100);
    float decimal = degrees + (minutes / 60.0);
    if (direction == "S" || direction == "W") decimal *= -1.0;
    return decimal;
}
void imuToCarFrame(float imuX, float imuY, float imuZ, float &carX, float &carY, float &carZ) {
    carX = imuZ;       // IMU Z -> Car X (forward) 
    carY = -imuX;      // IMU X -> Car Y (left)
    carZ = imuY;       // IMU Y -> Car Z (up)
}
void processNMEALine(String line) {
    if (line.startsWith("$GPRMC") || line.startsWith("$GPGGA")) {
        int index = 0;
        String fields[15];  
        int last = 0;
        for (int i = 0; i < line.length(); i++) {
            if (line[i] == ',') {
                fields[index++] = line.substring(last, i);
                last = i + 1;
            }
        }
        fields[index] = line.substring(last); 

        if (fields[0].startsWith("$GPRMC") && fields[2] == "A") {
            gpsFix = true;
        } else if (fields[0].startsWith("$GPGGA") && fields[6].toInt() > 0) {
            gpsFix = true;
        }

        if (fields[3].length() > 0 && fields[5].length() > 0) {
            gpsLat = convertToDecimal(fields[3].toFloat(), fields[4]);
            gpsLon = convertToDecimal(fields[5].toFloat(), fields[6]);
        }
    }
}

void checkIMUCalibration() {
  if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
    uint8_t acc = sensorValue.un.rotationVector.accuracy;

    Serial.print("Calibration Accuracy: ");
    Serial.println(acc);  // 0 = Uncalibrated, 3 = Fully Calibrated
  }
}

void setup() {
    delay(2000);  
    Serial.begin(115200);
    
    // Initialize I2C for IMU
    Wire2.begin();
    pinMode(LED1_PIN, OUTPUT);
    setSyncProvider(get_teensy3_time);
    bool rtc_set = timeStatus() == timeSet && year() > 2021;
    
    
    if (!rtc_set) {
        Serial.println("Warning: Failed to sync time with RTC");
        logFile = SD.open("log_unknown_time.csv", FILE_WRITE);
    } else {
        sprintf(log_name, "log_%04d-%02d-%02d_%02d-%02d-%02d.csv", year(), month(), day(), hour(), minute(), second());
        logFile = SD.open(log_name, FILE_WRITE);
    }

    if(!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD failed!");
        while(true){
            digitalWrite(LED1_PIN, HIGH);
            delay(250);
            digitalWrite(LED1_PIN, LOW);
            delay(250);
        }
    }
    
    if (!SD.exists(log_name)) {
        logFile = SD.open(log_name, FILE_WRITE);
    if (logFile) {
        logFile.println("Timestamp,CarRoll,CarPitch,CarYaw,RollGrad,ax,ay,az,ForwardAccel,LateralAccel,HeaveAccel,PitchRate,RollRate,YawRate,qw,qx,qy,qz,ShockPot1,ShockPot2");
        logFile.close();
    } else {
        Serial.println("Failed to create new log file");
        while(true){
            digitalWrite(LED2_PIN, HIGH);
            delay(250);
            digitalWrite(LED2_PIN, LOW);
            delay(250);
        }
        logFail = true;
        while(true);
    }
}

    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 initialization failed!");
        while (true);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 20);
    display.println("Initializing IMU...");
    display.display();

    // Initialize IMU
    if (!bno08x.begin_I2C(0x4A, &Wire2)) {
        Serial.println("BNO085 initialization failed!");
        display.println("IMU Error!");
        display.display();
        while(true){
            digitalWrite(LED3_PIN, HIGH);
            delay(250);
            digitalWrite(LED3_PIN, LOW);
            delay(250);
        }
        imuFail = true;
        while (true);
    }
    bno08x.enableReport(SH2_CAL_ACCEL, 10000);
    bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);

    // Initialize GPS
    myusb.begin();
    
    display.clearDisplay();
    display.setCursor(10, 20);
    display.println("IMU + GPS Ready!");
    display.display();
    delay(1000);
}


void loop() {
    myusb.Task();
    // read from usb GPS
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        Serial.write(c);
        if (c == '\n') {
            processNMEALine(lastLine);
            lastLine = "";
        } else if (c != '\r') {
            lastLine += c;
        }
    }
    // IMU data
  
    while (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_CAL_ACCEL: {
                float ax_imu = sensorValue.un.accelerometer.x;
                float ay_imu = sensorValue.un.accelerometer.y;
                float az_imu = sensorValue.un.accelerometer.z;
                imuToCarFrame(ax_imu, ay_imu, az_imu, ax, ay, az);
                lateralAccel = ay;
            break;
            }

            case SH2_ROTATION_VECTOR: {
                //Quaternions in IMU frame
                float qw = sensorValue.un.rotationVector.real;
                float qx = sensorValue.un.rotationVector.i;
                float qy = sensorValue.un.rotationVector.j;
                float qz = sensorValue.un.rotationVector.k;
                uint8_t acc = sensorValue.un.rotationVector.accuracy;
                // Serial.print("ACCURACY: ");
                // Serial.println(acc);

                //transformation quaternion (depends on sensor mounting)
                float ow = 0.5, ox = 0.5, oy = -0.5, oz = 0.5;

                //quaternion multiplication (Carq = fixedq * imuQ)
                rw = ow * qw - ox * qx - oy * qy - oz * qz;
                rx = ow * qx + ox * qw + oy * qz - oz * qy;
                ry = ow * qy - ox * qz + oy * qw + oz * qx;
                rz = ow * qz + ox * qy - oy * qx + oz * qw;

                //roation around car's x-axis aka side to side tilt
                roll  = atan2(2.0 * (rw * rx + ry * rz), 1.0 - 2.0 * (rx * rx + ry * ry)) * 180.0 / PI;
                float sinp = 2.0 * (rw * ry - rz * rx);
                sinp = constrain(sinp, -1.0, 1.0);
                //rotation around car's y-axis aka front to back tilt
                pitch = asin(sinp) * 180.0 / PI;
                //rotation around car's z-axis aka heading
                yaw   = atan2(2.0 * (rw * rz + rx * ry), 1.0 - 2.0 * (ry * ry + rz * rz)) * 180.0 / PI;
                break;
            }

            case SH2_LINEAR_ACCELERATION: {
                float lax_imu = sensorValue.un.linearAcceleration.x;
                float lay_imu = sensorValue.un.linearAcceleration.y;
                float laz_imu = sensorValue.un.linearAcceleration.z;
                float lax, lay, laz;
                imuToCarFrame(lax_imu, lay_imu, laz_imu, lax, lay, laz);
                forwardAccel = lax;
                lateralAccel = lay;
                heaveAccel = laz;
                break;
            }

            case SH2_GYROSCOPE_CALIBRATED: {
                float gx_imu = sensorValue.un.gyroscope.x;
                float gy_imu = sensorValue.un.gyroscope.y;
                float gz_imu = sensorValue.un.gyroscope.z;
                float gx, gy, gz;
                imuToCarFrame(gx_imu, gy_imu, gz_imu, gx, gy, gz);
                pitchRate = gx * RAD_TO_DEG;   // Rotation around car X (forward)
                rollRate  = gy * RAD_TO_DEG;   // Rotation around car Y (left)
                yawRate   = gz * RAD_TO_DEG;   // Rotation around car Z (up)

                break;
            }
        }
    }

    
    if (millis() - lastUpdate > UPDATE_INTERVAL) {
        lastUpdate = millis();
        float rollGradient = (fabs(lateralAccel) > 0.01) ? pitch / lateralAccel : 0.0;
        
        // hall sensor
        hall = analogRead(A1);
        float voltage_h = ((hall / 1023.0) * 3.3);
        float deg = (voltage_h / 3.3) * 360.0;
        
        //shock pot
        shock_pot = analogRead(A2);
        float voltage_sp = ((shock_pot / 1023.0) * 3.3);
        float distance = (voltage_sp / 3.3) * 250;

        //shock pot2
        shock_pot2 = analogRead(A13);
        float voltage_sp2 = ((shock_pot2 / 1023.0) * 3.3);
        float distance2 = (voltage_sp2 / 3.3) * 250;
      
        

        // Display on OLED
        display.clearDisplay();
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.println("Data!");
        if (gpsFix) {
            display.print("Lat: "); display.println(gpsLat, 6);
            display.print("Lon: "); display.println(gpsLon, 6);
            Serial.print(gpsLat, 6);
            Serial.println(gpsLon, 6);
            
        } else {
            display.println("Waiting for GPS fix...");
        }
        display.print("Roll: "); 
        display.println(roll, 1);
        display.print("lat acc: "); 
        display.println(lateralAccel, 2);
        display.print("RG: "); 
        display.println(rollGradient, 2);
        display.print("Deg: ");
        display.println(deg);
        display.display();

        //serial prints
        char timestamp[32];
        unsigned long ms = millis();
        time_t now = ms / 1000;
        int ms_part = ms % 1000;
        sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
        year(now), month(now), day(now),
        hour(now), minute(now), second(now), ms_part);

        //Serial.print(timestamp);
        Serial.print(" | CarRoll: "); Serial.print(roll, 1);
        Serial.print(" | CarPitch: "); Serial.print(pitch, 1);
        Serial.print(" | CarYaw: "); Serial.print(yaw, 1);

        //Serial.print(" | RollGrad: "); Serial.print(rollGradient, 2);

        // Serial.print(" | ax: "); Serial.print(ax, 2);
        // Serial.print(" | ay: "); Serial.print(ay, 2);
        // Serial.print(" | az: "); Serial.print(az, 2);

        Serial.print(" | ForAcc: "); Serial.print(forwardAccel, 2);
        Serial.print(" | LatAcc: "); Serial.print(lateralAccel, 2);
        Serial.print(" | HAcc: "); Serial.print(heaveAccel, 2);

        Serial.print(" | PiRate: "); Serial.print(pitchRate, 2);
        Serial.print(" | RoRate: "); Serial.print(rollRate, 2);
        Serial.print(" | YaRate: "); Serial.print(yawRate, 2);

        Serial.print(" | q_car = [");
        Serial.print(rw, 4); Serial.print(", ");
        Serial.print(rx, 4); Serial.print(", ");
        Serial.print(ry, 4); Serial.print(", ");
        Serial.println(rz, 4); Serial.print("]");

        // Serial.print(" | ShockPot1: "); Serial.print(distance, 1);
        // Serial.print(" | ShockPot2: "); Serial.println(distance2, 1);

        //logging to sd card
        logFile = SD.open(log_name, FILE_WRITE);
        if (logFile) {
            //logFile.print(timestamp); logFile.print(",");
            logFile.print(timestamp); logFile.print(",");

            logFile.print(roll); logFile.print(",");
            logFile.print(pitch); logFile.print(",");
            logFile.print(yaw); logFile.print(",");

            logFile.print(rollGradient); logFile.print(",");

            logFile.print(ax); logFile.print(",");
            logFile.print(ay); logFile.print(",");
            logFile.print(az); logFile.print(",");

            logFile.print(forwardAccel); logFile.print(",");
            logFile.print(lateralAccel); logFile.print(",");
            logFile.print(heaveAccel); logFile.print(",");

            logFile.print(pitchRate); logFile.print(",");
            logFile.print(rollRate); logFile.print(",");
            logFile.print(yawRate); logFile.print(",");

            logFile.print(rw); logFile.print(",");
            logFile.print(rx); logFile.print(",");
            logFile.print(ry); logFile.print(",");
            logFile.print(rz); logFile.print(",");

            logFile.print(distance); logFile.print(",");
            logFile.println(distance2); 

            logFile.close();  
        } else {
            Serial.println("Failed to write to log");
        }

    }
}
