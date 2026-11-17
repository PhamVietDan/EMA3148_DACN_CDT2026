// Minimal assertion helper shared by the host test files.
#pragma once

#include <stdio.h>

extern int failures;

#define CHECK(cond, ...)                                  \
    do {                                                  \
        if (!(cond)) {                                    \
            failures++;                                   \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);   \
            printf(__VA_ARGS__);                          \
            printf("\n");                                 \
        }                                                 \
    } while (0)

void test_arming(void);
void test_ctrl(void);
void test_params_all(void);
void test_alt(void);
void test_alt_hold(void);
void test_levelcal(void);
void test_yaw_hold(void);
void test_flow_est(void);
void test_pos_hold(void);
void test_ubx(void);
