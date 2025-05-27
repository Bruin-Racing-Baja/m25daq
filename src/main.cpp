#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_GPS.h>
#include <USBHost_t36.h>
#include <can_bus.h>
#include <constants.h>
#include <can_ids.h>
#include <queue>

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
#define GPS_UPDATE_INTERVAL 100
uint32_t lastGPSUpdate = 0;

#define LED1_PIN 4
#define LED2_PIN 9
#define LED3_PIN 10

String lastLine = "";
float gpsLat = 0.0;
float gpsLon = 0.0;
bool gpsFix = false;
int hall;
int shock_pot;
float roll = 0;
float pitch = 0;
float yaw = 0;
float lateralAccel = 0;
float ax = 0;
float ay = 0;
float az = 0;
float linAccel = 0;
float gyx = 0;
time_t get_teensy3_time() { return Teensy3Clock.get(); }
char log_name[32];
bool sdFail = false;
bool logFail = false;
bool imuFail = false;

File logFile;int cycle_count = 0;

Can_Bus bus;
CAN_message_t create_can_msg(u32 func_id, u32 node_id,  u32 sync_val, float data){
  CAN_message_t msg;
  u8 buf[8];
  msg.buf[0] = static_cast<u8>(func_id);
  msg.buf[1] = static_cast<u8>(sync_val);
  msg.buf[6] = 3;
  msg.id = (node_id << 5) | 0x1F;
  msg.len = 7;
  memcpy(msg.buf + 2, &data, 4);
  msg.flags.remote = false;
  return msg;
}
CAN_message_t create_can_msg(u32 func_id, u32 node_id, u32 sync_val, uint32_t data){
  CAN_message_t msg;
  u8 buf[8];
  msg.buf[0] = static_cast<u8>(func_id);
  msg.buf[1] = static_cast<u8>(sync_val);
  msg.buf[6] = 2;
  msg.id = (node_id << 5) | 0x1F;
  msg.len = 7;
  memcpy(msg.buf + 2, &data, 4);
  msg.flags.remote = false;
  return msg;
}


// Convert NMEA lat/lon to decimal degrees
float convertToDecimal(float nmeaCoord, String direction) {
    int degrees = int(nmeaCoord / 100);
    float minutes = nmeaCoord - (degrees * 100);
    float decimal = degrees + (minutes / 60.0);
    if (direction == "S" || direction == "W") decimal *= -1.0;
    return decimal;
}

void processNMEALine(String line) {
    if (line.startsWith("$GPRMC") || line.startsWith("$GPGGA")) {
        // Split by commas
        int index = 0;
        String fields[15];  // enough for typical sentence
        int last = 0;
        for (int i = 0; i < line.length(); i++) {
            if (line[i] == ',') {
                fields[index++] = line.substring(last, i);
                last = i + 1;
            }
        }
        fields[index] = line.substring(last);  // last field

        // Check fix validity
        if (fields[0].startsWith("$GPRMC") && fields[2] == "A") {
            gpsFix = true;
        } else if (fields[0].startsWith("$GPGGA") && fields[6].toInt() > 0) {
            gpsFix = true;
        }

        // Parse latitude and longitude
        if (fields[3].length() > 0 && fields[5].length() > 0) {
            gpsLat = convertToDecimal(fields[3].toFloat(), fields[4]);
            gpsLon = convertToDecimal(fields[5].toFloat(), fields[6]);
        }
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
        while (true) {
            sdFail = true;
        }
    }
    
    // if(!logFile) {
    //     Serial.println("File open failed!");
    //     Serial.println(logFile);
    // }
    // logFile.println("Timestamp,Roll,Pitch,Yaw,LatAccel,Ay,Az,LinAccel,RGrad,GyX");
    // logFile.close();
    if (!SD.exists(log_name)) {
        logFile = SD.open(log_name, FILE_WRITE);
    if (logFile) {
        logFile.println("Timestamp,CarPitch,CarRoll,CarYaw,LatAccel,Ay,ForwardAccel,LinAccel,RGrad,ShockPot,GyX");
        logFile.close();
    } else {
        Serial.println("Failed to create new log file");
        while (true) {
            logFail = true;
        }
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
        while (true) {
            imuFail = true;
        }
        while (true);
    }
    bno08x.enableReport(SH2_CAL_ACCEL, 10000);
    bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);

    // Initialize GPS
    myusb.begin();
    
    //InitializE Can Bus
    bus.setup();

    display.clearDisplay();
    display.setCursor(10, 20);
    display.println("IMU + GPS Ready!");
    display.display();
    delay(1000);
}


void loop() {
    if (sdFail) {
        digitalWrite(LED1_PIN, HIGH);
        delay(250);  
        digitalWrite(LED1_PIN, LOW);
        delay(250);  
    }
    if (logFail) {
        digitalWrite(LED2_PIN, HIGH);
        delay(250);  
        digitalWrite(LED2_PIN, LOW);
        delay(250);  
    }
    if (imuFail) {
        digitalWrite(LED3_PIN, HIGH);
        delay(250);  
        digitalWrite(LED3_PIN, LOW);
        delay(250);  
    }
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
    
    // Update display every gps interval
    if (millis() - lastGPSUpdate > GPS_UPDATE_INTERVAL) {
        lastGPSUpdate = millis();
        
        // IMU: read accelerometer
        while (bno08x.getSensorEvent(&sensorValue)) {
          switch (sensorValue.sensorId) {
            case SH2_CAL_ACCEL: {
              ax = sensorValue.un.accelerometer.x;
              ay = sensorValue.un.accelerometer.y;
              //lateral accel in car reference frame az
              az = sensorValue.un.accelerometer.z;
              lateralAccel = ax;
              break;
            }
            case SH2_ROTATION_VECTOR: {
              float qw = sensorValue.un.rotationVector.real;
              float qx = sensorValue.un.rotationVector.i;
              float qy = sensorValue.un.rotationVector.j;
              float qz = sensorValue.un.rotationVector.k;
            //   Serial.print(qw);
            //   Serial.print(qx);
            //   Serial.print(qy);
            //   Serial.println(qz);
              roll  = ((atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))) * 180.0) / PI;
              pitch = asin(2.0 * (qw * qy - qz * qx)) * 180.0 / PI;
              yaw   = ((atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))) * 180.0) / PI;
              //Serial.println(roll);
              break;
            }
            case SH2_LINEAR_ACCELERATION: {
                linAccel = sensorValue.un.linearAcceleration.z;
            }
            case SH2_GYROSCOPE_CALIBRATED: {
                gyx = sensorValue.un.gyroscope.x;
            }
          }
        }
        
        float rollGradient = (fabs(lateralAccel) > 0.01) ? pitch / lateralAccel : 0.0;
        
        //test hall sensor
        hall = analogRead(A1);
        float voltage_h = ((hall / 1023.0) * 3.3);
        float deg = (voltage_h / 3.3) * 360.0;
        
        shock_pot = analogRead(A2);
        float voltage_sp = ((shock_pot / 1023.0) * 3.3);
        float distance = (voltage_sp / 3.3) * 250;
      
        

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

        //Serial
        // Serial.print("Time (ms): "); 
        // Serial.print(millis());
        char timestamp[32];
        sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d.%03lu",
        year(), month(), day(), hour(), minute(), second(), millis() % 1000);
        Serial.print(timestamp);
        //Serial.println(micros());
        Serial.print(" Roll: ");
        Serial.print(roll);
        Serial.print(" Pitch: ");
        Serial.print(pitch);
        Serial.print(" Yaw: ");
        Serial.print(yaw);
        Serial.print(" lat accel "); 
        Serial.print(ax);
        Serial.print(" ay "); 
        Serial.print(ay);
        Serial.print(" az "); 
        Serial.print(az);
        Serial.print(" linz "); 
        Serial.print(linAccel);
        Serial.print(" RGrad: "); 
        Serial.print(rollGradient);
        Serial.print(" Shock Pot: "); 
        Serial.println(distance);
        Serial.print(" gy x: ");
        Serial.println(gyx);

        logFile = SD.open(log_name, FILE_WRITE);
        if (logFile) {
            //logFile.print(timestamp); logFile.print(",");
            logFile.print(timestamp); logFile.print(",");
            logFile.print(roll); logFile.print(",");
            logFile.print(pitch); logFile.print(",");
            logFile.print(yaw); logFile.print(",");
            logFile.print(ax); logFile.print(",");
            logFile.print(ay); logFile.print(",");
            logFile.print(az); logFile.print(",");
            logFile.print(linAccel); logFile.print(",");
            logFile.print(rollGradient); logFile.print(",");
            logFile.print(distance); logFile.print(",");
            logFile.println(gyx);
            logFile.close();  
        } else {
            Serial.println("Failed to write to log");
        }

        bus.send_command(create_can_msg(CAN_REAL_TIME, RASP_NODE_ID, cycle_count, (u32) now()));
        bus.send_command(create_can_msg(CAN_ROLL, RASP_NODE_ID, cycle_count, roll));
        bus.send_command(create_can_msg(CAN_PITCH, RASP_NODE_ID, cycle_count, pitch));
        bus.send_command(create_can_msg(CAN_YAW, RASP_NODE_ID, cycle_count, yaw));
        bus.send_command(create_can_msg(CAN_AX, RASP_NODE_ID, cycle_count, ax));
        bus.send_command(create_can_msg(CAN_AY, RASP_NODE_ID, cycle_count, ay));
        bus.send_command(create_can_msg(CAN_AZ, RASP_NODE_ID, cycle_count, az));
        bus.send_command(create_can_msg(CAN_LIN_ACCEL, RASP_NODE_ID, cycle_count, linAccel));
        bus.send_command(create_can_msg(CAN_ROLL_GRADIENT, RASP_NODE_ID, cycle_count, rollGradient));
        bus.send_command(create_can_msg(CAN_DISTANCE, RASP_NODE_ID, cycle_count, distance));
        bus.send_command(create_can_msg(CAN_GYX, RASP_NODE_ID, cycle_count, gyx));

        cycle_count++;
    }
}
