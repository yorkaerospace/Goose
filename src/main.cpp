#include <Arduino.h>
#include <String.h>

// Communication
#include <Wire.h>
#include <SPI.h>

// Components
#include <SD.h>
#define SPI_SD_SS  10
File SD_file;
File csv_file;

#include <MPU6050.h>
#define MPU6050_RANGE_2_G  0
#define MPU6050_RANGE_4_G  1
#define MPU6050_RANGE_8_G  2
#define MPU6050_RANGE_16_G 3

#define MPU6050_RANGE_250_DEG  0
#define MPU6050_RANGE_500_DEG  1
#define MPU6050_RANGE_1000_DEG 2
#define MPU6050_RANGE_2000_DEG 3

MPU6050 mpu_6050;
#define SELECTED_MPU6050_ACCEL_RANGE MPU6050_RANGE_2_G     // SETTINGS
#define SELECTED_MPU6050_GYRO_RANGE  MPU6050_RANGE_500_DEG // SETTINGS


#include <SparkFunLIS3DH.h>
#define LIS3DH_I2C_ADDRESS 0x19
LIS3DH lis3dh(I2C_MODE, LIS3DH_I2C_ADDRESS);
#define SELECTED_LIS3DH_ACCEL_RANGE 16 // 16, 8, 4, 2     // SETTINGS


#include <Adafruit_BMP280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
#define BMP280_I2C_ADDRESS 0x76
Adafruit_BMP280 bmp_280; //(&Wire)


#include <SoftwareSerial.h>
#include <TinyGPS.h>
TinyGPS gps;
// GPS_SERIAL_TX is left unused. The gps uses 3.3V, so we 
// can use it to recieve data from it, but 
// if we try to transmit data to it 
// we would definately damage the chip
#define GPS_SERIAL_TX   3 
#define GPS_SERIAL_RX   4 // GPS TX Pin Goes Here
#define GPS_POWER_SUPPLY_ENABLE 5
uint8_t gps_enabled = false;
SoftwareSerial gps_serial(GPS_SERIAL_RX, GPS_SERIAL_TX);

#define LOG_MINUTES 5

//868MHz


// Others

/*
 * Just for test and optimization purposess.
 * It makes the code to measure the speed of the readings 
 * of the sensors and prints it on the serial terminal 
 * every time the log_ function is called.
 * 
 * DO NOT USE IN PRODUCTION!
 * IT MAY SLOW THE CODE!
 */
//#define ANALYSE_READING_SENSORS_SPEED


////////////////////////////////////////////////////////////
// Data Logging and Buffer
////////////////////////////////////////////////////////////

struct data_t // 26 Bytes
{
    unsigned long timestamp;
    
    /*
     * Chip: MPU6050
     * The more accurate accelerometer,
     * it will be used to sense the smaller changes in the acceleration.
     */
    int16_t x_accelerometer_6050;
    int16_t y_accelerometer_6050;
    int16_t z_accelerometer_6050;

    int16_t x_gyroscope_6050;
    int16_t y_gyroscope_6050;
    int16_t z_gyroscope_6050;

    /*
     * Chip: LIS3DH
     * The more inaccurate accelerometer,
     * it will be used to sense the high acceleration 
     * on the vertical axis of the rocket.
     */
    int16_t x_accelerometer_lis3dh;
    int16_t y_accelerometer_lis3dh;
    int16_t z_accelerometer_lis3dh;

    float pressure;
    float altitude;

    // We can add some flags.
    // It makes the struct perfect 32 bytes.
    uint16_t flags;
};

#define BUFFER_SIZE 1
uint8_t buffer_element_position = 0;

/*
 * The buffer should be 512 bytes in order 
 * to be written in the SD card really fast.
 */
struct data_t data_buffer[BUFFER_SIZE];

/*
 * Keep in mind, that data here is not a pointer 
 * like in the log_ functions.
 * 
 * Also this function doesn't check for overflow,
 * so buffer_element_position should 
 * be monitored independently.
 */
void write_data_to_buffer(struct data_t data)
{
    data_buffer[buffer_element_position] = data;
    ++buffer_element_position;
}

uint8_t buffer_is_full(void)
{
    return buffer_element_position >= BUFFER_SIZE;
}

/*
 * Keep in mind, that data here is not a pointer 
 * like in the log_ functions.
 */
void print_data_struct(struct data_t data)
{
    Serial.println("\n----------");

    Serial.print("Timestamp: ");
    Serial.println(data.timestamp);

    Serial.print("Accelerometer (MPU6050), x = ");
    Serial.print(data.x_accelerometer_6050);
    Serial.print(", y = ");
    Serial.print(data.y_accelerometer_6050);
    Serial.print(", z = ");
    Serial.println(data.z_accelerometer_6050);

    Serial.print("Gyroscope (MPU6050), x = ");
    Serial.print(data.x_gyroscope_6050);
    Serial.print(", y = ");
    Serial.print(data.y_gyroscope_6050);
    Serial.print(", z = ");
    Serial.println(data.x_gyroscope_6050);

    Serial.print("Accelerometer (LIS3DH), x = ");
    Serial.print(data.x_accelerometer_lis3dh);
    Serial.print(", y = ");
    Serial.print(data.y_accelerometer_lis3dh);
    Serial.print(", z = ");
    Serial.println(data.z_accelerometer_lis3dh);

    Serial.print("Barometer (BMP280), pressure = ");
    Serial.print(data.pressure);
    Serial.print(", altitude = ");
    Serial.println(data.altitude);

    // TODO: print flags
}


////////////////////////////////////////////////////////////
// Utils
////////////////////////////////////////////////////////////

void print_status(uint8_t status)
{
    if (status)
    {
        Serial.println("success");
    }
    else
    {
        Serial.println("failure");
    }
}


////////////////////////////////////////////////////////////
// SD Card Functions
////////////////////////////////////////////////////////////

void init_sd_card(void)
{
    Serial.print("SD Init begun: ");

    print_status(
        (uint8_t)SD.begin(SPI_SD_SS)
    );
}

void open_raw_file_random_name()
{
    randomSeed(analogRead(0));
    char filename[] = "____.txt";
    filename[0] = random('a', 'z');
    filename[1] = random('a', 'z');
    filename[2] = random('a', 'z');
    filename[3] = random('a', 'z');

    SD_file = SD.open(filename, FILE_WRITE);
    if (!SD_file)
    {
        Serial.println("Failed to open file!");
    }
}

void open_raw_file()
{
    SD_file = SD.open("asd.txt", FILE_WRITE);
    if (!SD_file)
    {
        Serial.println("Failed to open file!");
    }
}

void close_file(void)
{
    SD_file.close();
}

void store_buffer(void)
{
    if (SD_file)
    {
        SD_file.write((const uint8_t *)data_buffer, sizeof(struct data_t)*BUFFER_SIZE);
    }
}

void read_data()
{
    SD_file = SD.open("asd.txt");
    if (!SD_file)
    {
        Serial.println("Failed to open file!");
    }

    unsigned long counter = 0;
    while(SD_file.available() && counter < 80)
    {
        struct data_t data;
        SD_file.read((uint8_t *)&data, sizeof(struct data_t));

        print_data_struct(data);
        ++counter;
    }

    close_file();

    Serial.print("Number of readings: ");
    Serial.println(counter);
}


////////////////////////////////////////////////////////////
// MPU6050 Functions
////////////////////////////////////////////////////////////

void init_mpu_6050(void)
{
    Serial.print("6050 Init begun: ");
    mpu_6050.initialize();

    print_status(
        (uint8_t)mpu_6050.testConnection()
    );

    mpu_6050.setFullScaleAccelRange(SELECTED_MPU6050_ACCEL_RANGE);
    mpu_6050.setFullScaleGyroRange(SELECTED_MPU6050_GYRO_RANGE);
}

void log_accelerometer_6050(struct data_t *data)
{
#ifdef ANALYSE_READING_SENSORS_SPEED
    unsigned long start_time = micros();
#endif

    mpu_6050.getAcceleration(
        &data->x_accelerometer_6050, 
        &data->y_accelerometer_6050, 
        &data->z_accelerometer_6050
    );

#ifdef ANALYSE_READING_SENSORS_SPEED
    unsigned long end_time = micros();

    Serial.print("Accelerometer 6050 reading speed: ");
    Serial.print(end_time - start_time);
    Serial.println(" microseconds");
#endif
}

void log_gyroscope_6050(struct data_t *data)
{
#ifdef ANALYSE_READING_SENSORS_SPEED
    unsigned long start_time = micros();
#endif

    mpu_6050.getRotation(
        &data->x_gyroscope_6050, 
        &data->y_gyroscope_6050, 
        &data->z_gyroscope_6050
    );

#ifdef ANALYSE_READING_SENSORS_SPEED
    unsigned long end_time = micros();

    Serial.print("Gyroscope 6050 reading speed: ");
    Serial.print(end_time - start_time);
    Serial.println(" microseconds");
#endif
}

float get_accelerometer_scale_factor_6050(void)
{
    switch (SELECTED_MPU6050_ACCEL_RANGE)
    {
        case MPU6050_RANGE_16_G:
            return 2048;
        case MPU6050_RANGE_8_G:
            return 4096;
        case MPU6050_RANGE_4_G:
            return 8192;
        case MPU6050_RANGE_2_G:
            return 16384;
    }
}

float get_gyroscope_scale_factor_6050(void)
{
    switch (SELECTED_MPU6050_GYRO_RANGE)
    {
        case MPU6050_RANGE_250_DEG:
            return 131;
        case MPU6050_RANGE_500_DEG:
            return 65.5;
        case MPU6050_RANGE_1000_DEG:
            return 32.8;
        case MPU6050_RANGE_2000_DEG:
            return 16.4;
    }
}


////////////////////////////////////////////////////////////
// LIS3DH Functions
////////////////////////////////////////////////////////////

void init_lis3dh(void)
{
    // TODO: Check the configuration
    // 1600 probably could also do the job.
    lis3dh.settings.accelSampleRate = 5000; 
    lis3dh.settings.accelRange = SELECTED_LIS3DH_ACCEL_RANGE;

    Serial.print("LIS3DH Init begun: ");

    print_status(
        (uint8_t)lis3dh.begin()
    );
}

/*
 * This function reads raw data from the accelerometer,
 * that is stored as int16_t (2 bytes) in order to save memory.
 * When we translate the stored data into a human readable format,
 * we will use:
 *     " float LIS3DH::calcAccel( int16_t input ) " 
 * in order to calculate the real acceleration 
 * relative to the acceletation range.
 */
void log_accelerometer_lis3dh(struct data_t *data)
{
#ifdef ANALYSE_READING_SENSORS_SPEED
    unsigned long start_time = micros();
#endif

    // I have no idea why these were multiplied by 10.
    // TODO: Ask Dany or Rebecca.
    data->x_accelerometer_lis3dh = lis3dh.readRawAccelX(); // * 10;
    data->y_accelerometer_lis3dh = lis3dh.readRawAccelY(); // * 10;
    data->z_accelerometer_lis3dh = lis3dh.readRawAccelZ(); // * 10;

#ifdef ANALYSE_READING_SENSORS_SPEED
    unsigned long end_time = micros();

    Serial.print("Accelerometer lis3dh reading speed: ");
    Serial.print(end_time - start_time);
    Serial.println(" microseconds");
#endif
}

////////////////////////////////////////////////////////////
// BMP280 Functions
////////////////////////////////////////////////////////////

void init_bmp280(void)
{
    Serial.print("BMP_280 Init begun: ");

    print_status((uint8_t)
        bmp_280.begin(BMP280_I2C_ADDRESS)
    );
}

void log_barometer_bmp280(struct data_t *data)
{
#ifdef ANALYSE_READING_SENSORS_SPEED
    unsigned long start_time = micros();
#endif

    data->pressure = bmp_280.readPressure();

    data->altitude = 44330 * (1.0 - pow( (data->pressure / 100) / SEALEVELPRESSURE_HPA, 0.1903));
    
    /*
     * readAltitude() reads pressure first and then 
     * calculates the above calculations to get an altituse. 
     * This second reading slows the code.
     * So we will use our previous reading to calculate it.
     */
    //data->altitude = bmp_280.readAltitude(SEALEVELPRESSURE_HPA);

#ifdef ANALYSE_READING_SENSORS_SPEED
    unsigned long end_time = micros();

    Serial.print("Barometer BMP280 reading speed: ");
    Serial.print(end_time - start_time);
    Serial.println(" microseconds");
#endif
}

////////////////////////////////////////////////////////////
// GPS Functions
////////////////////////////////////////////////////////////

struct location_t 
{
    float latitude;
    float longitude;
};

void init_gps(void)
{
    pinMode(GPS_SERIAL_RX, INPUT);
    pinMode(GPS_POWER_SUPPLY_ENABLE, OUTPUT);

    // In the beginning the gps should be disabled
    digitalWrite(GPS_POWER_SUPPLY_ENABLE, LOW);
    gps_enabled = false;
}

void start_gps()
{
    gps_serial.begin(9600);
    digitalWrite(GPS_POWER_SUPPLY_ENABLE, HIGH);
    gps_enabled = true;
}

void stop_gps()
{
    digitalWrite(GPS_POWER_SUPPLY_ENABLE, LOW);
    gps_serial.end();
    gps_enabled = false;
}

struct location_t get_location(void)
{
    bool newData = false;
    struct location_t location = {0};

    if (!gps_enabled)
    {
        return location;
    }

    // For one second we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
        while (gps_serial.available())
        {
            char c = gps_serial.read();
            if (gps.encode(c)) // Did a new valid sentence come in?
                newData = true;
        }
    }

    if (newData)
    {   
        unsigned long age;
        gps.f_get_position(&location.latitude, &location.longitude, &age);

        if (location.latitude == TinyGPS::GPS_INVALID_F_ANGLE)
        {
            location.latitude = 0;
        }

        if (location.longitude == TinyGPS::GPS_INVALID_F_ANGLE)
        {
            location.longitude = 0;
        }
    }

    return location;
}

////////////////////////////////////////////////////////////
// CSV File Functions
////////////////////////////////////////////////////////////

void translate_data_to_csv()
{
    SD_file = SD.open("asd.txt", O_READ);
    csv_file = SD.open("real.txt", FILE_WRITE);

    if (!SD_file)
    {
        Serial.println("Failed to open SD_file file!");
        return;
    }

    if (!csv_file)
    {
        Serial.println("Failed to open csv_file file!");
        return;
    }

    float mpu6050_accel_scale = get_accelerometer_scale_factor_6050();
    float mpu6050_gyro_scale = get_gyroscope_scale_factor_6050();

    csv_file.println("microseconds, x_acc_6050, y_acc_6050, z_acc_6050, x_gyro, y_gyro, z_gyro, x_acc_lis3dh, y_acc_lis3dh, z_acc_lis3dh, pressure, altitude, flags");

    unsigned long counter = 0;
    while(SD_file.available())
    {
        struct data_t data;
        SD_file.read((uint8_t *)&data, sizeof(struct data_t));

        String buffer = String(data.timestamp) + ", ";

            buffer += String( (float)data.x_accelerometer_6050 / mpu6050_accel_scale ) + ", ";
            buffer += String( (float)data.y_accelerometer_6050 / mpu6050_accel_scale ) + ", ";
            buffer += String( (float)data.z_accelerometer_6050 / mpu6050_accel_scale ) + ", ";

            buffer += String( (float)data.x_gyroscope_6050 / mpu6050_gyro_scale ) + ", ";
            buffer += String( (float)data.y_gyroscope_6050 / mpu6050_gyro_scale ) + ", ";
            buffer += String( (float)data.z_gyroscope_6050 / mpu6050_gyro_scale ) + ", ";

            buffer += String( lis3dh.calcAccel(data.x_accelerometer_lis3dh) ) + ", ";
            buffer += String( lis3dh.calcAccel(data.y_accelerometer_lis3dh) ) + ", ";
            buffer += String( lis3dh.calcAccel(data.z_accelerometer_lis3dh) ) + ", ";

            buffer += String(data.pressure) + ", ";
            buffer += String(data.altitude) + ", ";
            buffer += String(data.flags);

        Serial.println(buffer);
        csv_file.println(buffer);
        ++counter;
    }

    close_file();
    csv_file.close();

    Serial.print("Number of readings: ");
    Serial.println(counter);
}

////////////////////////////////////////////////////////////
// Control Functions
////////////////////////////////////////////////////////////
void point_of_apogy();
void is_landed();
void user_mode();

float wait_for_launch()
{
    struct data_t data;

    log_accelerometer_6050(&data);
    log_gyroscope_6050(&data);
    log_accelerometer_lis3dh(&data);

    Serial.print( lis3dh.calcAccel(data.x_accelerometer_lis3dh) );
    Serial.print("     ");
    Serial.println((float)data.y_accelerometer_6050/get_accelerometer_scale_factor_6050());

    if (fabsf(lis3dh.calcAccel(data.x_accelerometer_lis3dh)) > 6)
    {
        Serial.println("Launch");
    }

    return fabsf(lis3dh.calcAccel(data.x_accelerometer_lis3dh));
}

void test_log_10_seconds()
{
    open_raw_file();
    //open_raw_file_random_name();

    unsigned long t1 = millis();
    unsigned long t2 = millis();
    unsigned long counter = 0;

    unsigned long time_of_barometer_last_reading = 0;
    float pressure = 0;
    float altitude = 0;

    while(t2 - t1 < 60000 * LOG_MINUTES)
    {
        struct data_t data;

        //----- Read the sensor data -----

        data.timestamp = micros();
        log_accelerometer_6050(&data);
        log_gyroscope_6050(&data);
        log_accelerometer_lis3dh(&data);

        // There is no point of reading the barometer really 
        // fast, because it works at 157Hz internally.
        // So the fast consecuent measurement 
        // would be the same as the previous.
        if (data.timestamp - time_of_barometer_last_reading > 20000)
        {
            log_barometer_bmp280(&data);
            time_of_barometer_last_reading = data.timestamp;
            pressure = data.pressure;
            altitude = data.altitude;
        }
        else
        {
            data.pressure = pressure;
            data.altitude = altitude;
        }

        write_data_to_buffer(data);
        ++counter;

        if (buffer_is_full())
        {
            store_buffer();
            buffer_element_position = 0;
        }

        t2 = millis();
    }

    close_file();

    Serial.print("Number of readings: ");
    Serial.println(counter);
    Serial.println("Ready");
}

void log_data_forever()
{
    open_raw_file();

    unsigned long time_of_barometer_last_reading = 0;
    float pressure = 0;
    float altitude = 0;

    unsigned long t1 = millis();

    while(true)
    {
        struct data_t data;

        //----- Read the sensor data -----

        data.timestamp = micros();
        log_accelerometer_6050(&data);
        log_gyroscope_6050(&data);
        log_accelerometer_lis3dh(&data);

        // There is no point of reading the barometer really 
        // fast, because it works at 157Hz internally.
        // So the fast consecuent measurement 
        // would be the same as the previous.
        if (data.timestamp - time_of_barometer_last_reading > 20000)
        {
            log_barometer_bmp280(&data);
            time_of_barometer_last_reading = data.timestamp;
            pressure = data.pressure;
            altitude = data.altitude;
        }
        else
        {
            data.pressure = pressure;
            data.altitude = altitude;
        }

        write_data_to_buffer(data);

        if (buffer_is_full())
        {
            store_buffer();
            buffer_element_position = 0;
        }

        if (millis() - t1 > 10000)
        {
            close_file();
            delay(10);
            open_raw_file();
            t1 = millis();
        }
    }

    close_file();
}

void setup() 
{
    delay(2000); // Just for test purposes
    Serial.begin(9600); // 115200
    Serial.println("Start...");
    Wire.begin();

    //----- Init the sensors -----
    init_mpu_6050();
    init_lis3dh();
    init_bmp280();

    init_sd_card();

    //init_gps();

    if (SD.remove("asd.txt") == true)
    {
        Serial.println("File removed successfully");
    }

    Wire.setClock(1000000);
    SPI.setClockDivider(SPI_CLOCK_DIV2);

    delay(10000);
    Serial.println("Start");
}

void loop() 
{
    
    //delay(1000);
    
    //open_raw_file();
    test_log_10_seconds();

    //Serial.println("End Logging");


    //open_raw_file_random_name();
    //log_data_forever();
    //close_file();

    //translate_data_to_csv();

    // translate_data_to_csv();

    // Serial.println("End Translation");
    //delay(1000);

    while(1);
}
