/*
 * Standalone specification-gap probe; deliberately NOT part of run_host_tests.
 * An expected nonzero exit status demonstrates MOTOR-151 is not yet implemented.
 * The pending diagnostic is observational: no timeout has been agreed.
 */
#include "balancing_core.h"

#include <stdint.h>
#include <stdio.h>

static BModeInput ready_input(void)
{
    BModeInput input = {0};
    input.fresh.imu = true;
    input.fresh.power = true;
    for (unsigned i = 0; i < 4; ++i) input.fresh.rs[i] = true;
    for (unsigned i = 0; i < 2; ++i) input.fresh.wheel[i] = true;
    input.power_on = true;
    input.calibration_verified = true;
    input.model_verified = true;
    input.limits_verified = true;
    input.posture_ok = true;
    return input;
}

int main(void)
{
    BModeMachine machine;
    BModeInput input = ready_input();
    BModeOutput output;
    unsigned mismatches = 0;

    b_mode_init(&machine);
    input.arm_edge = true;
    input.posture_ok = false;
    output = b_mode_step(&machine, &input, UINT64_C(1000000));
    if (!output.request_rs_enable || !machine.arm_pending) {
        puts("MOTOR-151: SPEC MISMATCH: posture gate rejected otherwise ready ARM");
        ++mismatches;
    } else {
        puts("MOTOR-151: spec behavior confirmed");
    }

    b_mode_init(&machine);
    input = ready_input();
    input.arm_edge = true;
    output = b_mode_step(&machine, &input, UINT64_C(2000000));
    input.arm_edge = false;
    output = b_mode_step(&machine, &input, UINT64_C(4000000));
    printf("MOTOR-198: DIAGNOSTIC only (timeout not specified): "
           "after 2s without replies, mode=%d pending=%d disable=%d\n",
           output.mode, machine.arm_pending, output.request_rs_disable);

    return mismatches == 0 ? 0 : 1;
}
