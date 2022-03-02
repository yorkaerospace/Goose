#include <Arduino.h>
#include <String.h>

// Communication
#include <Wire.h>
#include <SPI.h>

// Components
#include <SD.h>
#include "MPU6050.h"
#include "SparkFunLIS3DH.h"
#include "Adafruit_BMP280.h"

#define SEALEVELPRESSURE_HPA (1013.25)

// Add/remove fields as is appropriate
struct values
{
    int16_t acc_6050[3];
    int16_t gyr_6050[3];
    int16_t acc_LIS3[3];
    
    float bar_280[3];

} data;

uint8_t SPI_SD_SS = 10;
File SD_file;

uint8_t SPI_Rad_SS = 7;

int32_t count;

MPU6050 mpu_6050;
LIS3DH mpu_LIS3(I2C_MODE, 0x19);
Adafruit_BMP280 bmp_280;

void setup() 
{
    count = 0;
    
    Wire.begin();
    Serial.begin(9600);

    Serial.println("Serial Init success");
    Serial.print("SD Init begun: ");
    Serial.println(SD.begin(SPI_SD_SS) ? "success" : "failed");

    Serial.print("6050 Init begun: ");
    mpu_6050.initialize();
    Serial.println(mpu_6050.testConnection() ? "success" : "failure");

    mpu_LIS3.settings.accelSampleRate = 5000;
    mpu_LIS3.settings.accelRange = 16;
    Serial.print("LIS3DH Init begun: ");
    Serial.println(mpu_LIS3.begin() ? "failure": "success");

    Serial.print("BMP_280 Init begun: ");
    Serial.println(bmp_280.begin(0x76) ? "success" : "failure");
    // auto status = bmp_280.begin(); 
    // You can also pass in a Wire library object like &Wire2
    // status = bmp.begin(0x76, &Wire2)
    // if (!status) {
    //     Serial.println("Could not find a valid BMP280 sensor, check wiring, address, sensor ID!");
    //     Serial.print("SensorID was: 0x"); Serial.println(bmp_280.sensorID(),16);
    //     Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    //     Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    //     Serial.print("        ID of 0x60 represents a BMP 280.\n");
    //     Serial.print("        ID of 0x61 represents a BMP 680.\n");
    //     // while (1) delay(10);
    // }
    Serial.print(bmp_280.readTemperature());
    Serial.print(", ");
    Serial.print(bmp_280.readPressure());
    Serial.print(", ");
    Serial.println(bmp_280.readAltitude(SEALEVELPRESSURE_HPA));


    delay(1000);
    Serial.println("Time ");
    delay(1000);
}

void loop() 
{    
    // primary MPU (MPU6050)
    unsigned long t1 = millis();
    mpu_6050.getMotion6(&data.acc_6050[0], &data.acc_6050[1], &data.acc_6050[2], &data.gyr_6050[0], &data.gyr_6050[1], &data.gyr_6050[2]);

    // secondary accelorometer (LIS3DH)
    data.acc_LIS3[0] = mpu_LIS3.readRawAccelX() * 10;
    data.acc_LIS3[1] = mpu_LIS3.readRawAccelY() * 10;
    data.acc_LIS3[2] = mpu_LIS3.readRawAccelZ() * 10;

    // barometer (BMP-280)
    data.bar_280[0] = bmp_280.readTemperature();
    data.bar_280[1] = bmp_280.readPressure();
    data.bar_280[2] = bmp_280.readAltitude(SEALEVELPRESSURE_HPA);

    //String asd = String(data.acc_6050[0]);
    String buffer = String(count) + ", " + String(millis()) + ", ";
    buffer += String(data.acc_6050[0]) + ", " + String(data.acc_6050[1]) + ", " + String(data.acc_6050[2]) + ", ";
    buffer +=  String(data.gyr_6050[0]) + ", " + String(data.gyr_6050[1]) + ", " + String(data.gyr_6050[2]) + ", ";
    buffer += String(data.acc_LIS3[0]) + ", " + String(data.acc_LIS3[1]) + ", " + String(data.acc_LIS3[2]) + ", ";
    buffer += String(data.bar_280[0]) + ", " + String(data.bar_280[1]) + ", " + String(data.bar_280[2]);

    Serial.println(buffer);

    // burst write to SD card
    if (count % 10 == 0)
    {
        SD_file = SD.open("log.txt", FILE_WRITE);

        // init_dump = SD.open("extra.txt", FILE_WRITE);
        if (SD_file)
        {
            SD_file.println(buffer);
        }
        else
            Serial.println("Failed to SD File");
        SD_file.close();
        
    }
    count++;

    unsigned long t2 = millis();
    Serial.println(t2 - t1);
}