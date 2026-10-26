// Host tests for App/control_logic/arming.c: arm/disarm rules, RC-loss failsafe, spool state (PLAN §7.7, §12).
#include "control_logic/arming.h"
#include "control_logic/config.h"
#include "test_common.h"

// Simulated vehicle at 1 kHz (task_ctrl runs at ~1067 Hz).
typedef struct {
    arming_t a;
    arming_in_t in;
    arming_out_t out;
    int armed_events, disarmed_events, blocked_events, fs1_events, recovered_events;
    uint8_t last_disarm;
} sim_t;

static void sim_init(sim_t *s)
{
    *s = (sim_t){0};
    arming_init(&s->a);
    s->in.now_ms = 1000;
    s->in.rc_valid = true;
    s->in.imu_ok = true;
    s->in.calib_done = true;
    s->in.lc_angle = 30.0f * MATHX_DEG2RAD;
    s->in.lc_ms = 1000;
}

// Advances ms milliseconds with the current inputs. gcs_disarm is consumed on the first step.
static void run(sim_t *s, uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        s->in.now_ms++;
        arming_update(&s->a, &s->in, &s->out);
        s->in.gcs_disarm = false;
        switch (s->out.event) {
        case ARM_EV_ARMED:        s->armed_events++; break;
        case ARM_EV_DISARMED:     s->disarmed_events++; s->last_disarm = s->out.event_arg; break;
        case ARM_EV_BLOCKED:      s->blocked_events++; break;
        case ARM_EV_FS_STAGE1:    s->fs1_events++; break;
        case ARM_EV_FS_RECOVERED: s->recovered_events++; break;
        default: break;
        }
    }
}

// Switch to ARM after a settled DISARM position, i.e. a real edge.
static void arm_by_switch(sim_t *s)
{
    s->in.arm_sw = false;
    run(s, 300);
    s->in.arm_sw = true;
    run(s, 300);
}

static void test_boot_switch_on_does_not_arm(void)
{
    sim_t s;
    sim_init(&s);
    s.in.arm_sw = true;  // already in ARM at power-up
    run(&s, 2000);
    CHECK(!s.out.armed && s.armed_events == 0, "armed from the power-up switch position");
}

static void test_switch_arm_disarm(void)
{
    sim_t s;
    sim_init(&s);
    arm_by_switch(&s);
    CHECK(s.out.armed && s.armed_events == 1, "switch edge did not arm");
    CHECK(s.out.motor_state == MOTOR_IDLE, "not idle after arming (%d)", s.out.motor_state);
    s.in.arm_sw = false;
    run(&s, 300);
    CHECK(!s.out.armed && s.last_disarm == DISARM_SWITCH, "switch off did not disarm");
    CHECK(s.out.motor_state == MOTOR_STOP, "motors not stopped after disarm");
}

static void test_switch_debounce(void)
{
    sim_t s;
    sim_init(&s);
    s.in.arm_sw = false;
    run(&s, 300);
    s.in.arm_sw = true;  // glitch shorter than the debounce
    run(&s, 100);
    s.in.arm_sw = false;
    run(&s, 300);
    CHECK(!s.out.armed && s.armed_events == 0 && s.blocked_events == 0, "100 ms switch glitch acted");
}

static void test_arm_checks(void)
{
    sim_t s;
    sim_init(&s);
    s.in.rc_thr = 0.2f;
    arm_by_switch(&s);
    CHECK(!s.out.armed && s.blocked_events == 1, "armed with throttle up");
    CHECK(s.out.block_mask & ARM_BLK_THROTTLE, "throttle block bit missing");

    sim_init(&s);
    s.in.pitch = 20.0f * MATHX_DEG2RAD;
    arm_by_switch(&s);
    CHECK(!s.out.armed && (s.out.block_mask & ARM_BLK_ANGLE), "armed while tilted 20 deg");

    sim_init(&s);
    s.in.calib_done = false;
    arm_by_switch(&s);
    CHECK(!s.out.armed && (s.out.block_mask & ARM_BLK_CALIB), "armed before calibration");

    sim_init(&s);
    s.in.imu_ok = false;
    arm_by_switch(&s);
    CHECK(!s.out.armed && (s.out.block_mask & ARM_BLK_IMU), "armed without IMU");

    sim_init(&s);
    s.in.rc_roll = 0.5f;
    arm_by_switch(&s);
    CHECK(!s.out.armed && (s.out.block_mask & ARM_BLK_STICKS), "armed with roll stick deflected");
}

static void test_stick_gestures(void)
{
    sim_t s;
    sim_init(&s);
    s.in.arm_sw = true;  // switch already in ARM at power-up, throttle low, yaw right: must not arm
    run(&s, 300);
    s.in.rc_yaw = 1.0f;
    run(&s, 5000);
    CHECK(!s.out.armed && s.armed_events == 0 && s.blocked_events == 0, "a stick gesture armed");

    sim_init(&s);
    arm_by_switch(&s);
    CHECK(s.out.armed, "switch did not arm");
    s.in.rc_yaw = 0.0f;
    run(&s, 100);
    s.in.rc_yaw = -1.0f;
    run(&s, 3100);
    CHECK(!s.out.armed && s.last_disarm == DISARM_STICK, "stick disarm failed");
}

static void test_stick_disarm_needs_low_throttle(void)
{
    sim_t s;
    sim_init(&s);
    arm_by_switch(&s);
    s.in.rc_thr = 0.5f;
    s.in.rc_yaw = -1.0f;
    run(&s, 5000);
    CHECK(s.out.armed, "stick disarm with throttle up (would cut motors in flight)");
}

static void test_failsafe(void)
{
    sim_t s;
    sim_init(&s);
    arm_by_switch(&s);
    s.in.rc_thr = 0.4f;
    s.in.rc_roll = 0.3f;
    run(&s, 1000);
    CHECK(s.out.motor_state == MOTOR_RUN, "not running at 40 %% throttle (%d)", s.out.motor_state);

    // Receiver failsafe: frames keep coming but rc_valid drops; CH5 jumps to the middle (= DISARM).
    s.in.rc_valid = false;
    s.in.arm_sw = false;
    s.in.rc_thr = 0.0f;
    run(&s, 150);
    CHECK(s.out.armed && s.out.fs == FS_NONE, "early stage change at 150 ms");
    CHECK(s.out.thr_cmd > 0.39f && s.out.roll_cmd > 0.29f, "last sticks not held during a short gap");
    run(&s, 100);
    CHECK(s.out.fs == FS_STAGE1 && s.fs1_events == 1, "stage 1 not entered after 200 ms");
    CHECK(s.out.roll_cmd == 0.0f && s.out.thr_cmd > 0.39f, "stage 1 must level and hold throttle");
    CHECK(s.out.armed, "receiver failsafe switch value disarmed immediately");
    run(&s, 800);
    CHECK(!s.out.armed && s.last_disarm == DISARM_FAILSAFE, "stage 2 did not disarm at 1 s");

    // Link back with the switch still in ARM: no re-arm until the switch is cycled.
    s.in.rc_valid = true;
    s.in.arm_sw = true;
    s.in.rc_roll = 0.0f;
    run(&s, 1000);
    CHECK(!s.out.armed && (s.out.block_mask & ARM_BLK_SWITCH_CYCLE), "re-armed after failsafe without switch cycle");
    arm_by_switch(&s);
    CHECK(s.out.armed, "switch cycle did not allow re-arming");
}

static void test_failsafe_recovery(void)
{
    sim_t s;
    sim_init(&s);
    arm_by_switch(&s);
    s.in.rc_thr = 0.4f;
    run(&s, 1000);
    s.in.rc_valid = false;
    run(&s, 400);
    CHECK(s.out.fs == FS_STAGE1, "stage 1 expected");
    s.in.rc_valid = true;
    s.in.rc_thr = 0.6f;
    run(&s, 300);
    CHECK(s.out.fs == FS_STAGE1 && s.out.thr_cmd < 0.41f, "recovered before 0.5 s of good RC");
    run(&s, 300);
    CHECK(s.out.fs == FS_NONE && s.recovered_events == 1 && s.out.armed, "no recovery after 0.5 s");
    CHECK(s.out.thr_cmd > 0.59f, "sticks not live again after recovery");
}

static void test_auto_and_fault_disarms(void)
{
    sim_t s;
    sim_init(&s);
    arm_by_switch(&s);
    run(&s, CFG_AUTO_DISARM_MS + 10);
    CHECK(!s.out.armed && s.last_disarm == DISARM_AUTO, "no auto-disarm at idle");

    sim_init(&s);
    arm_by_switch(&s);
    s.in.gcs_disarm = true;
    run(&s, 1);
    CHECK(!s.out.armed && s.last_disarm == DISARM_GCS, "GCS DISARM ignored");

    sim_init(&s);
    arm_by_switch(&s);
    s.in.imu_ok = false;
    run(&s, 1);
    CHECK(!s.out.armed && s.last_disarm == DISARM_IMU, "IMU loss did not disarm");

    sim_init(&s);
    arm_by_switch(&s);
    s.in.roll = 80.0f * MATHX_DEG2RAD;
    run(&s, 250);
    CHECK(s.out.armed, "flip disarm before 0.3 s");
    run(&s, 100);
    CHECK(!s.out.armed && s.last_disarm == DISARM_FLIP, "flip not detected");
}

static void test_spool(void)
{
    sim_t s;
    sim_init(&s);
    arm_by_switch(&s);
    s.in.rc_thr = 0.5f;
    run(&s, 250);
    CHECK(s.out.motor_state == MOTOR_SPOOL_UP && s.out.thr_ceiling > 0.4f && s.out.thr_ceiling < 0.6f,
          "spool-up ramp: state %d ceiling %f", s.out.motor_state, (double)s.out.thr_ceiling);
    run(&s, 300);
    CHECK(s.out.motor_state == MOTOR_RUN && s.out.thr_ceiling == 1.0f, "not RUN after spool-up");
    s.in.rc_thr = 0.0f;
    run(&s, 1);
    CHECK(s.out.motor_state == MOTOR_IDLE, "throttle at minimum should return to IDLE");
}

static void test_lost_control(void)
{
    sim_t s;
    sim_init(&s);
    arm_by_switch(&s);
    s.in.att_err = 40.0f * MATHX_DEG2RAD;  // at idle throttle: not flying, must not trip
    run(&s, 2000);
    CHECK(s.out.armed, "lost-control tripped at idle");

    s.in.rc_thr = 0.4f;
    s.in.att_err = 0.0f;
    run(&s, 1000);
    s.in.att_err = 40.0f * MATHX_DEG2RAD;  // short excursion (gust / overshoot)
    run(&s, 600);
    s.in.att_err = 5.0f * MATHX_DEG2RAD;
    run(&s, 100);
    s.in.att_err = 40.0f * MATHX_DEG2RAD;
    run(&s, 600);
    CHECK(s.out.armed, "two 0.6 s excursions tripped a 1 s check");
    run(&s, 450);
    CHECK(!s.out.armed && s.last_disarm == DISARM_LOST_CONTROL, "not disarmed after > 1 s far from setpoint");
    CHECK(s.out.block_mask & ARM_BLK_SWITCH_CYCLE, "lost-control disarm must require a switch cycle");
}

void test_arming(void)
{
    test_lost_control();
    test_boot_switch_on_does_not_arm();
    test_switch_arm_disarm();
    test_switch_debounce();
    test_arm_checks();
    test_stick_gestures();
    test_stick_disarm_needs_low_throttle();
    test_failsafe();
    test_failsafe_recovery();
    test_auto_and_fault_disarms();
    test_spool();
}
