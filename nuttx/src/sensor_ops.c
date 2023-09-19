#include <stdio.h>

#include "sensor_ops.h"

void read_imu(int fd, struct imu_msg_float *imu_data) {
  int16_t raw_data[7];
  struct imu_msg raw_msg;

  int ret = read(fd, &raw_data, sizeof(raw_data));
  if (ret != sizeof(raw_data)) {
    printf("Failed to read accelerometer data\n");
  } else {
    raw_msg.acc_x = ((raw_data[0] & REG_HIGH_MASK) << 8) +
                      ((raw_data[0] & REG_LOW_MASK) >> 8);
    imu_data->acc_x = raw_msg.acc_x / (float)CONFIG_APPLICATION_ENGINE_SPEED_AFS_SEL;
  }
}
