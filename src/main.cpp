#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_GPS.h>
#include <USBHost_t36.h>

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
#define GPS_UPDATE_INTERVAL 1000
uint32_t lastGPSUpdate = 0;

String lastLine = "";
float gpsLat = 0.0;
float gpsLon = 0.0;
bool gpsFix = false;
int hall;

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
        while (true);
    }
    bno08x.enableReport(SH2_CAL_ACCEL, 10000);
    bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);

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
    
    // Update display every gps interval
    if (millis() - lastGPSUpdate > GPS_UPDATE_INTERVAL) {
        lastGPSUpdate = millis();
        float roll = 0;
        float lateralAccel = 0;
        // IMU: read accelerometer
        float x = 0, y = 0, z = 0;
        if (bno08x.getSensorEvent(&sensorValue)) {
          switch (sensorValue.sensorId) {
            case SH2_CAL_ACCEL: {
              float ax = sensorValue.un.accelerometer.x;
              float ay = sensorValue.un.accelerometer.y;
              //lateral accel in car reference frame az
              float az = sensorValue.un.accelerometer.z;
              lateralAccel = az;
              break;
            }
            case SH2_ROTATION_VECTOR: {
              float qw = sensorValue.un.rotationVector.real;
              float qx = sensorValue.un.rotationVector.i;
              float qy = sensorValue.un.rotationVector.j;
              float qz = sensorValue.un.rotationVector.k;

              roll  = ((atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))) * 180.0) / PI;
              break;
            }
          }
        }
        
        float rollGradient = (fabs(lateralAccel) > 0.01) ? roll / lateralAccel : 0.0;
        
        //test hall sensor
        hall = analogRead(A1);
        float voltage = ((hall / 1023.0) * 3.3);
        float deg = (voltage / 3.3) * 360.0;
      
        

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
        display.print("az: "); 
        display.println(lateralAccel, 2);
        display.print("RG: "); 
        display.println(rollGradient, 2);
        display.print("Deg: ");
        display.println(deg);
        display.display();

        //Serial
        Serial.print("Time (ms): "); Serial.print(millis());
        Serial.print("Roll: ");
        Serial.print(roll);
        Serial.print(" az"); 
        Serial.print(lateralAccel);
        Serial.print(" RGrad: "); 
        Serial.print(rollGradient);
        Serial.print(" Hall: "); 
        Serial.println(deg);
    }
}
