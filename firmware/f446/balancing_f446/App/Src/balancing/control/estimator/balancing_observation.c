#include "balancing/control/estimator/balancing_observation.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define B_OBS_ENCODER_COUNTS 8192
#define B_OBS_HALF_TURN 4096
#define B_OBS_TAU 6.2831853071795864769

static bool valid_stamp(BSampleStamp old, uint64_t sample_us, uint64_t now_us)
{
    return sample_us <= now_us && (!old.valid || sample_us > old.stamp_us);
}

static bool valid_imu(const BImuSample *sample)
{
    if (sample == NULL) {
        return false;
    }
    for (unsigned axis = 0u; axis < 3u; ++axis) {
        if (!isfinite(sample->accel_m_s2[axis]) ||
            !isfinite(sample->gyro_rad_s[axis])) {
            return false;
        }
    }
    return true;
}

static bool valid_wheel_config(const BWheelObservationConfig *config)
{
    return config != NULL && config->verified &&
           isfinite(config->wheel_radius_m) &&
           isfinite(config->motor_revs_per_wheel_rev) &&
           (config->direction == 1.0f || config->direction == -1.0f) &&
           config->wheel_radius_m > 0.0f &&
           config->motor_revs_per_wheel_rev > 0.0f;
}

void b_observation_init(BObservationStore *store)
{
    if (store != NULL) {
        memset(store, 0, sizeof(*store));
    }
}

bool b_observation_imu(BObservationStore *store, const BImuSample *value,
                       bool new_measurement_proven, uint64_t stamp_us,
                       uint64_t now_us)
{
    if (store == NULL || !new_measurement_proven || !valid_imu(value) ||
        !valid_stamp(store->stamps.imu, stamp_us, now_us)) {
        return false;
    }
    store->imu = *value;
    store->stamps.imu = (BSampleStamp){true, stamp_us};
    return true;
}

bool b_observation_wheel(BObservationStore *store, unsigned wheel_index,
                         const BWheelObservationConfig *config,
                         uint16_t encoder_raw, int16_t rpm,
                         uint64_t stamp_us, uint64_t now_us)
{
    if (store == NULL || wheel_index >= 2u || !valid_wheel_config(config) ||
        encoder_raw >= B_OBS_ENCODER_COUNTS ||
        !valid_stamp(store->stamps.wheel[wheel_index], stamp_us, now_us)) {
        return false;
    }

    /* The algorithm cannot reconstruct extra motor revolutions between
     * feedback frames. Callers must validate actual speed/update cadence. */
    const BSampleStamp old_stamp = store->stamps.wheel[wheel_index];
    const BWheelObservation old = store->wheel[wheel_index];
    if (old_stamp.valid) {
        const BWheelObservationConfig *saved = &store->wheel_config[wheel_index];
        if (saved->wheel_radius_m != config->wheel_radius_m ||
            saved->motor_revs_per_wheel_rev != config->motor_revs_per_wheel_rev ||
            saved->direction != config->direction) {
            return false;
        }
    }
    int counts = 0;
    if (old_stamp.valid) {
        counts = (int)encoder_raw - (int)old.encoder_raw;
        if (counts > B_OBS_HALF_TURN) {
            counts -= B_OBS_ENCODER_COUNTS;
        } else if (counts < -B_OBS_HALF_TURN) {
            counts += B_OBS_ENCODER_COUNTS;
        }
        if (counts == B_OBS_HALF_TURN || counts == -B_OBS_HALF_TURN) {
            return false; /* exactly half a revolution is direction-ambiguous */
        }
    }

    const double meters_per_rotor_turn =
        B_OBS_TAU * (double)config->wheel_radius_m /
        (double)config->motor_revs_per_wheel_rev * (double)config->direction;
    const double position =
        (double)old.position_m + meters_per_rotor_turn * (double)counts /
        (double)B_OBS_ENCODER_COUNTS;
    const double velocity = meters_per_rotor_turn * (double)rpm / 60.0;
    if (!isfinite(meters_per_rotor_turn) ||
        !isfinite(position) || !isfinite(velocity) ||
        fabs(position) > FLT_MAX || fabs(velocity) > FLT_MAX) {
        return false;
    }

    BWheelObservation updated = {
        .encoder_raw = encoder_raw,
        .rpm = rpm,
        .position_m = (float)position,
        .velocity_m_s = (float)velocity
    };
    store->wheel[wheel_index] = updated;
    store->wheel_config[wheel_index] = *config;
    store->stamps.wheel[wheel_index] = (BSampleStamp){true, stamp_us};
    return true;
}

bool b_observation_freeze(const BObservationStore *store, uint64_t now_us,
                          BObservationSnapshot *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    memset(snapshot, 0, sizeof(*snapshot));
    if (store == NULL) {
        return false;
    }
    if ((store->stamps.imu.valid && store->stamps.imu.stamp_us > now_us) ||
        (store->stamps.power.valid && store->stamps.power.stamp_us > now_us)) {
        return false;
    }
    for (unsigned i = 0u; i < 4u; ++i) {
        if (store->stamps.rs[i].valid && store->stamps.rs[i].stamp_us > now_us) {
            return false;
        }
    }
    for (unsigned i = 0u; i < 2u; ++i) {
        if (store->stamps.wheel[i].valid && store->stamps.wheel[i].stamp_us > now_us) {
            return false;
        }
    }

    snapshot->stamps = store->stamps;
    snapshot->imu = store->imu;
    for (unsigned i = 0u; i < 2u; ++i) {
        snapshot->wheel[i] = store->wheel[i];
    }
    snapshot->frozen_at_us = now_us;
    return true;
}
