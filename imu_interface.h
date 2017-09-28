#ifndef __IMU_INTERFACE_H__
#define __IMU_INTERFACE_H__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (IMU related interface definition)
*******************************************************************************/
#include <stdint.h>

struct IMU_filtered_data_t {
    int32_t r1;
    int32_t r2;
    int32_t r3;
    int32_t r4;
    int32_t r5;
    int32_t r6;
    int32_t r7;
    int32_t r8;
    int32_t r9;
    int32_t r10;
    int32_t r11;
    int32_t r12;

    int32_t q1;
    int32_t q2;
    int32_t q3;
    int32_t q4;
    int32_t q5;
    int32_t q6;
    int32_t q7;
    int32_t q8;
    int32_t q9;
    int32_t q10;
    int32_t q11;
    int32_t q12;
} __attribute__((packed));

#endif  // __IMU_INTERFACE__
