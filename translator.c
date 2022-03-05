#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#pragma pack(2)

struct data_t // 26 Bytes
{
  uint32_t timestamp;
    
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

int main()
{
  FILE *ptr_bin;
  FILE *ptr_csv;
  struct data_t data;

  ptr_csv = fopen("real1.txt","w");
  ptr_bin = fopen("ASD1.TXT","rb");

  fprintf(ptr_csv,"microseconds, x_acc_6050, y_acc_6050, z_acc_6050, x_gyro, y_gyro, z_gyro, x_acc_lis3dh, y_acc_lis3dh, z_acc_lis3dh, pressure, altitude, flags\n");


  unsigned long counter = 0;
  while ( fread(&data, sizeof(struct data_t), 1, ptr_bin) == 1 )
    {
      ++counter;
      fprintf(ptr_csv, "%u, ", data.timestamp);

      fprintf(ptr_csv, "%f, ", (float)data.x_accelerometer_6050 / 16384 );
      fprintf(ptr_csv, "%f, ", (float)data.y_accelerometer_6050 / 16384 );
      fprintf(ptr_csv, "%f, ", (float)data.z_accelerometer_6050 / 16384 );

      fprintf(ptr_csv, "%f, ", (float)data.x_gyroscope_6050 / 65.5 );
      fprintf(ptr_csv, "%f, ", (float)data.y_gyroscope_6050 / 65.5 );
      fprintf(ptr_csv, "%f, ", (float)data.z_gyroscope_6050 / 65.5 );

      fprintf(ptr_csv, "%f, ",  (float)data.x_accelerometer_lis3dh / 1280 );
      fprintf(ptr_csv, "%f, ",  (float)data.y_accelerometer_lis3dh / 1280 );
      fprintf(ptr_csv, "%f, ",  (float)data.z_accelerometer_lis3dh / 1280 );

      fprintf(ptr_csv, "%f, ", data.pressure);
      fprintf(ptr_csv, "%f, ", data.altitude);
      fprintf(ptr_csv, "%u\n", data.flags);
    }

  fclose(ptr_csv);
  fclose(ptr_bin);

  printf("Number of data entries = %lu\n", counter);
  return EXIT_SUCCESS;
}
