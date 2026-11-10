#include "balancing/control/estimator/balancing_leg_kinematics.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define B_LEG_PI 3.14159265358979323846f

static bool finite_pair(const float pair[2])
{
    return pair != NULL && isfinite(pair[0]) && isfinite(pair[1]);
}

static bool valid_motor_map(const BLegMotorMap *map)
{
    if (map == NULL || !map->calibration_valid ||
        !isfinite(map->driving_teeth) || !isfinite(map->driven_teeth) ||
        map->driving_teeth <= 0.0f || map->driven_teeth <= 0.0f) {
        return false;
    }
    for (unsigned i = 0; i < 2; ++i) {
        if (!isfinite(map->motor_zero_rad[i]) ||
            !isfinite(map->joint_offset_rad[i]) ||
            (map->sign[i] != 1.0f && map->sign[i] != -1.0f)) {
            return false;
        }
    }
    return true;
}

bool b_leg_motor_to_joint(const BLegMotorMap *map, const float motor_angle_rad[2],
                          const float motor_rate_rad_s[2], BLegJointState *joint)
{
    if (joint == NULL) {
        return false;
    }
    memset(joint, 0, sizeof(*joint));
    if (!valid_motor_map(map) || !finite_pair(motor_angle_rad) ||
        !finite_pair(motor_rate_rad_s)) {
        return false;
    }
    const float ratio = map->driving_teeth / map->driven_teeth;
    if (!isfinite(ratio) || ratio <= 0.0f) {
        return false;
    }
    for (unsigned i = 0; i < 2; ++i) {
        const float scale = ratio * map->sign[i];
        const float q = scale * (motor_angle_rad[i] - map->motor_zero_rad[i]) +
                        map->joint_offset_rad[i];
        const float qdot = scale * motor_rate_rad_s[i];
        if (!isfinite(q) || !isfinite(qdot)) {
            memset(joint, 0, sizeof(*joint));
            return false;
        }
        joint->q_rad[i] = q;
        joint->qdot_rad_s[i] = qdot;
    }
    return true;
}

static bool valid_geometry(const BLegFixtureGeometry *g)
{
    return g != NULL && isfinite(g->base_span_m) &&
           isfinite(g->left_crank_m) && isfinite(g->right_crank_m) &&
           isfinite(g->left_rod_m) && isfinite(g->right_rod_m) &&
           isfinite(g->min_link_cross_sine) &&
           isfinite(g->min_leg_length_m) &&
           g->base_span_m > 0.0f && g->left_crank_m > 0.0f &&
           g->right_crank_m > 0.0f && g->left_rod_m > 0.0f &&
           g->right_rod_m > 0.0f && g->min_link_cross_sine > 0.0f &&
           g->min_link_cross_sine < 1.0f && g->min_leg_length_m > 0.0f;
}

BLegStatus b_leg_fixture_forward(const BLegFixtureGeometry *g,
                                 BLegAssemblyBranch branch,
                                 const BLegJointState *joint, BLegKinematics *out)
{
    if (out == NULL) {
        return B_LEG_INVALID_INPUT;
    }
    memset(out, 0, sizeof(*out));
    if (!valid_geometry(g) || joint == NULL ||
        (branch != B_LEG_BRANCH_NEGATIVE && branch != B_LEG_BRANCH_POSITIVE) ||
        !finite_pair(joint->q_rad) || !finite_pair(joint->qdot_rad_s)) {
        return B_LEG_INVALID_INPUT;
    }

    /* The link closure below is a mathematical fixture with two actuated
     * base cranks and two distal rods meeting at a virtual foot. No claim
     * that the actual six-bar CAD topology has this circle intersection. */
    const float q0 = remainderf(joint->q_rad[0], 2.0f * B_LEG_PI);
    const float q1 = remainderf(joint->q_rad[1], 2.0f * B_LEG_PI);
    const float ex = g->left_crank_m * cosf(q0);
    const float ey = g->left_crank_m * sinf(q0);
    const float fx = g->base_span_m + g->right_crank_m * cosf(q1);
    const float fy = g->right_crank_m * sinf(q1);
    const float dx = fx - ex;
    const float dy = fy - ey;
    const float d = hypotf(dx, dy);
    if (!isfinite(d) || d <= 0.0f) {
        return B_LEG_SINGULAR;
    }
    const float left = g->left_rod_m;
    const float right = g->right_rod_m;
    const float reach_max = left + right;
    const float reach_min = fabsf(left - right);
    if (d > reach_max || d < reach_min) {
        return B_LEG_UNREACHABLE;
    }

    const float projection = (left * left - right * right + d * d) / (2.0f * d);
    float height_sq = left * left - projection * projection;
    if (!isfinite(height_sq)) {
        return B_LEG_INVALID_INPUT;
    }
    /* Near a circle tangency, reject the branch instead of dividing by a
     * singular Jacobian. The small epsilon only absorbs float rounding. */
    const float eps = 16.0f * 1.1920928955078125e-7f * (left * left + right * right);
    if (height_sq < -eps) {
        return B_LEG_UNREACHABLE;
    }
    height_sq = fmaxf(0.0f, height_sq);
    const float h = sqrtf(height_sq);
    const float ux = dx / d;
    const float uy = dy / d;
    const float side = (float)branch;
    const float px = ex + projection * ux + side * h * (-uy);
    const float py = ey + projection * uy + side * h * ux;
    const float c0x = px - ex;
    const float c0y = py - ey;
    const float c1x = px - fx;
    const float c1y = py - fy;
    const float determinant = c0x * c1y - c0y * c1x;
    const float cross_sine = fabsf(determinant) / (left * right);
    if (!isfinite(cross_sine) || cross_sine < g->min_link_cross_sine) {
        return B_LEG_SINGULAR;
    }

    const float leg_x = px - g->base_span_m * 0.5f;
    const float leg_y = py;
    const float length = hypotf(leg_x, leg_y);
    if (!isfinite(length) || length < g->min_leg_length_m) {
        return B_LEG_SINGULAR;
    }
    /* Implicit differentiation of |P-E|^2=left^2, |P-F|^2=right^2:
     * (P-E).dP=(P-E).dE and (P-F).dP=(P-F).dF. */
    const float rhs0 = c0x * (-g->left_crank_m * sinf(q0)) +
                       c0y * (g->left_crank_m * cosf(q0));
    const float rhs1 = c1x * (-g->right_crank_m * sinf(q1)) +
                       c1y * (g->right_crank_m * cosf(q1));
    const float jpx[2] = {rhs0 * c1y / determinant, -c0y * rhs1 / determinant};
    const float jpy[2] = {-rhs0 * c1x / determinant, c0x * rhs1 / determinant};

    BLegKinematics value = {0};
    value.foot_x_m = leg_x;
    value.foot_y_m = leg_y;
    value.length_m = length;
    value.theta_rad = atan2f(leg_x, -leg_y);
    const float inv_length = 1.0f / length;
    const float inv_length_sq = inv_length * inv_length;
    for (unsigned i = 0; i < 2; ++i) {
        value.jacobian.dlength_dq_m[i] =
            (leg_x * jpx[i] + leg_y * jpy[i]) * inv_length;
        value.jacobian.dtheta_dq[i] =
            (leg_x * jpy[i] - leg_y * jpx[i]) * inv_length_sq;
        value.length_rate_m_s += value.jacobian.dlength_dq_m[i] * joint->qdot_rad_s[i];
        value.theta_rate_rad_s += value.jacobian.dtheta_dq[i] * joint->qdot_rad_s[i];
    }
    if (!isfinite(value.foot_x_m) || !isfinite(value.foot_y_m) ||
        !isfinite(value.length_m) || !isfinite(value.theta_rad) ||
        !isfinite(value.length_rate_m_s) || !isfinite(value.theta_rate_rad_s) ||
        !finite_pair(value.jacobian.dlength_dq_m) ||
        !finite_pair(value.jacobian.dtheta_dq)) {
        return B_LEG_INVALID_INPUT;
    }
    *out = value;
    return B_LEG_OK;
}

bool b_leg_virtual_to_joint_torque(const BLegKinematics *state,
                                   const BLegVirtualLoad *load,
                                   BLegTorquePair *torque)
{
    if (torque == NULL) {
        return false;
    }
    memset(torque, 0, sizeof(*torque));
    if (state == NULL || load == NULL ||
        !isfinite(load->axial_force_n) || !isfinite(load->pitch_torque_nm) ||
        !finite_pair(state->jacobian.dlength_dq_m) ||
        !finite_pair(state->jacobian.dtheta_dq)) {
        return false;
    }
    for (unsigned i = 0; i < 2; ++i) {
        const float value = load->axial_force_n * state->jacobian.dlength_dq_m[i] +
                            load->pitch_torque_nm * state->jacobian.dtheta_dq[i];
        if (!isfinite(value)) {
            memset(torque, 0, sizeof(*torque));
            return false;
        }
        torque->torque_nm[i] = value;
    }
    return true;
}

bool b_leg_joint_to_ideal_motor_torque(const BLegMotorMap *map,
                                       const BLegTorquePair *joint_torque,
                                       BLegTorquePair *motor_torque)
{
    if (motor_torque == NULL) {
        return false;
    }
    memset(motor_torque, 0, sizeof(*motor_torque));
    if (!valid_motor_map(map) || joint_torque == NULL ||
        !finite_pair(joint_torque->torque_nm)) {
        return false;
    }
    const float ratio = map->driving_teeth / map->driven_teeth;
    for (unsigned i = 0; i < 2; ++i) {
        const float value = joint_torque->torque_nm[i] * map->sign[i] * ratio;
        if (!isfinite(value)) {
            memset(motor_torque, 0, sizeof(*motor_torque));
            return false;
        }
        motor_torque->torque_nm[i] = value;
    }
    return true;
}
