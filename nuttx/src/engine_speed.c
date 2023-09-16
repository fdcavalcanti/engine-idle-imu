/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <fcntl.h>
#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/ioctl.h>

#include "sensor_ops.h"
#include "gsl/gsl_vector.h"
#include <gsl/gsl_fft_real.h>
#include <gsl/gsl_statistics.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PREDICTION_VECTOR_LENGTH 128

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int fft(gsl_vector* acc_data, gsl_vector* fft_abs_vector);

/****************************************************************************
 * main
 ****************************************************************************/

int main(int argc, FAR char *argv[]) {
  printf("Starting engine speed estimator\n");
  struct imu_msg_float acc_msg;
  int counter = 0;
  float peak_frequency = -1;

  /* Open the IMU device driver */

  int fd = open("/dev/imu0", O_RDWR);
  if (fd < 0)
    {
      printf("Failed to open IMU\n");
      return EXIT_FAILURE;
    }

  /* Allocate the GSL vector to calculate FFT */

  gsl_vector* acc_x_vector = gsl_vector_calloc(PREDICTION_VECTOR_LENGTH);
  gsl_vector* fft_abs_vector = gsl_vector_calloc(PREDICTION_VECTOR_LENGTH);

  while (1)
    {
      read_imu(fd, &acc_msg);
      gsl_vector_set(acc_x_vector, counter, acc_msg.acc_x);
      counter++;
      if (counter == PREDICTION_VECTOR_LENGTH - 1)
      {
        counter = 0;
        fft(acc_x_vector, fft_abs_vector);
        peak_frequency = gsl_stats_max(fft_abs_vector->data, 1, PREDICTION_VECTOR_LENGTH);
        printf("Frequency peak [Hz]: %f\n", peak_frequency);
      }
    }
}


int fft(gsl_vector* acc_data, gsl_vector* fft_abs_vector) {
    // Copy the input data because of inplace operations
    float mean = gsl_stats_mean(acc_data->data, 1, acc_data->size);
    float scale = 1.0/(acc_data->size);

    gsl_vector_add_constant(acc_data, -mean);
    gsl_fft_real_radix2_transform(acc_data->data, 1, acc_data->size);
    gsl_vector_scale(acc_data, scale);

    for (size_t i=0; i < fft_abs_vector->size; i++) {
        float real, imag, absolute;
        real = acc_data->data[i];
        imag = acc_data->data[acc_data->size - i];
        absolute = sqrt(real*real + imag*imag);
        gsl_vector_set(fft_abs_vector, i, 2*absolute);
    }

    return 0;
}
