#include "leg_kinematics.h"

#include <math.h>

#define LEG_O1_Y_MM          21.0f
#define LEG_O2_X_ABS_MM      36.37306696f
#define LEG_L1_MM            24.0f
#define LEG_L2_MM           120.0f
#define LEG_L3_MM           120.0f
#define LEG_L4_MM            35.0f
#define LEG_CD_MM           155.0f
#define LEG_BD_MM           (LEG_CD_MM - LEG_L4_MM)

#define LEG_THETA1_MIN_DEG (-180.0f)
#define LEG_THETA1_MAX_DEG   25.0f
#define LEG_THETA2_MIN_DEG (-170.0f)
#define LEG_THETA2_MAX_DEG  (-45.0f)

#define LEG_FT_THETA_SPAN   205.0f

#define LEG_FT0_RAW_THETA_MAX   32.0f
#define LEG_FT0_RAW_THETA_MIN 1005.0f
#define LEG_FT3_RAW_THETA_MAX  999.0f
#define LEG_FT3_RAW_THETA_MIN   25.0f

#define LEG_THETA2_SPAN_DEG (LEG_THETA2_MAX_DEG - LEG_THETA2_MIN_DEG)
#define LEG_DM_RAD_MIN       0.0f
#define LEG_DM_RAD_MAX       2.22f
#define LEG_DM_RAD_SPAN      (LEG_DM_RAD_MAX - LEG_DM_RAD_MIN)
#define LEG_PI 3.14159265358979323846f

typedef struct {
    leg_point_t a;
    leg_point_t b;
    leg_point_t c;
    leg_point_t d;
    leg_joint_angles_t joint;
    uint8_t cross_count;
    uint8_t limit_count;
    float limit_penalty;
} leg_candidate_t;

static int leg_side_is_valid(leg_side_t side)
{
    return (side == LEG_SIDE_LEFT) || (side == LEG_SIDE_RIGHT);
}

static float deg_to_rad(float deg)
{
    return deg * LEG_PI / 180.0f;
}

static float rad_to_deg(float rad)
{
    return rad * 180.0f / LEG_PI;
}

static float clampf_local(float x, float min_value, float max_value)
{
    if (x < min_value) {
        return min_value;
    }
    if (x > max_value) {
        return max_value;
    }
    return x;
}

static uint16_t round_to_u16(float x, float min_value, float max_value)
{
    x = clampf_local(x, min_value, max_value);
    return (uint16_t)(x + 0.5f);
}

static leg_point_t point_make(float x, float y)
{
    leg_point_t p = {x, y};
    return p;
}

static void leg_get_origins(leg_side_t side, leg_point_t *o1, leg_point_t *o2)
{
    *o1 = point_make(0.0f, LEG_O1_Y_MM);
    *o2 = point_make((side == LEG_SIDE_LEFT) ? -LEG_O2_X_ABS_MM : LEG_O2_X_ABS_MM, 0.0f);
}

static float point_distance(leg_point_t p0, leg_point_t p1)
{
    float dx = p1.x_mm - p0.x_mm;
    float dy = p1.y_mm - p0.y_mm;
    return sqrtf(dx * dx + dy * dy);
}

static float cross2d(float ax, float ay, float bx, float by)
{
    return ax * by - ay * bx;
}

static uint8_t segment_intersects(leg_point_t p1, leg_point_t p2,
                                  leg_point_t p3, leg_point_t p4,
                                  uint8_t shared_endpoint)
{
    float d1x = p2.x_mm - p1.x_mm;
    float d1y = p2.y_mm - p1.y_mm;
    float d2x = p4.x_mm - p3.x_mm;
    float d2y = p4.y_mm - p3.y_mm;
    float denom = cross2d(d1x, d1y, d2x, d2y);

    if (fabsf(denom) < 1.0e-8f) {
        return 0U;
    }

    float dx = p3.x_mm - p1.x_mm;
    float dy = p3.y_mm - p1.y_mm;
    float t = cross2d(dx, dy, d2x, d2y) / denom;
    float u = cross2d(dx, dy, d1x, d1y) / denom;
    uint8_t t_internal = (t > 1.0e-4f) && (t < (1.0f - 1.0e-4f));
    uint8_t u_internal = (u > 1.0e-4f) && (u < (1.0f - 1.0e-4f));

    if (shared_endpoint != 0U) {
        return (uint8_t)(t_internal && u_internal);
    }

    return (uint8_t)((t >= -1.0e-8f) && (t <= (1.0f + 1.0e-8f)) &&
                     (u >= -1.0e-8f) && (u <= (1.0f + 1.0e-8f)) &&
                     t_internal && u_internal);
}

static uint8_t leg_cross_count(leg_point_t o1, leg_point_t a,
                               leg_point_t o2, leg_point_t b,
                               leg_point_t c, leg_point_t d)
{
    uint8_t count = 0U;

    count += segment_intersects(o1, a, o2, b, 0U);
    count += segment_intersects(o1, a, b, c, 0U);
    count += segment_intersects(o1, a, c, d, 0U);
    count += segment_intersects(o1, a, b, d, 0U);
    count += segment_intersects(o2, b, a, c, 0U);
    count += segment_intersects(o2, b, c, d, 0U);

    count += segment_intersects(o1, a, a, c, 1U);
    count += segment_intersects(o2, b, b, c, 1U);
    count += segment_intersects(o2, b, b, d, 1U);
    count += segment_intersects(a, c, b, c, 1U);
    count += segment_intersects(a, c, c, d, 1U);

    count += segment_intersects(a, c, b, d, 0U);

    return count;
}

static uint8_t angle_limit_count(leg_joint_angles_t joint)
{
    uint8_t count = 0U;
    const float eps = 1.0e-6f;

    if ((joint.theta1_deg < (LEG_THETA1_MIN_DEG - eps)) ||
        (joint.theta1_deg > (LEG_THETA1_MAX_DEG + eps))) {
        count++;
    }
    if ((joint.theta2_deg < (LEG_THETA2_MIN_DEG - eps)) ||
        (joint.theta2_deg > (LEG_THETA2_MAX_DEG + eps))) {
        count++;
    }

    return count;
}

static float angle_limit_penalty(leg_joint_angles_t joint)
{
    float penalty = 0.0f;
    const float eps = 1.0e-6f;

    if (joint.theta1_deg < (LEG_THETA1_MIN_DEG - eps)) {
        penalty += LEG_THETA1_MIN_DEG - joint.theta1_deg;
    } else if (joint.theta1_deg > (LEG_THETA1_MAX_DEG + eps)) {
        penalty += joint.theta1_deg - LEG_THETA1_MAX_DEG;
    }

    if (joint.theta2_deg < (LEG_THETA2_MIN_DEG - eps)) {
        penalty += LEG_THETA2_MIN_DEG - joint.theta2_deg;
    } else if (joint.theta2_deg > (LEG_THETA2_MAX_DEG + eps)) {
        penalty += joint.theta2_deg - LEG_THETA2_MAX_DEG;
    }

    return penalty;
}

static int candidate_is_better(const leg_candidate_t *candidate,
                               const leg_candidate_t *best)
{
    if (candidate->limit_count != best->limit_count) {
        return candidate->limit_count < best->limit_count;
    }
    if (candidate->limit_penalty != best->limit_penalty) {
        return candidate->limit_penalty < best->limit_penalty;
    }
    if (candidate->cross_count != best->cross_count) {
        return candidate->cross_count < best->cross_count;
    }
    return candidate->c.y_mm > best->c.y_mm;
}

static uint8_t circle_intersections(leg_point_t c0, float r0,
                                    leg_point_t c1, float r1,
                                    leg_point_t out[2])
{
    float dx = c1.x_mm - c0.x_mm;
    float dy = c1.y_mm - c0.y_mm;
    float dist = sqrtf(dx * dx + dy * dy);
    const float eps = 1.0e-8f;

    if ((dist < eps) || (dist > (r0 + r1 + eps)) ||
        (dist < (fabsf(r0 - r1) - eps))) {
        return 0U;
    }

    float a = (r0 * r0 - r1 * r1 + dist * dist) / (2.0f * dist);
    float h2 = r0 * r0 - a * a;
    if (h2 < -eps) {
        return 0U;
    }

    float h = sqrtf(fmaxf(0.0f, h2));
    float mx = c0.x_mm + a * dx / dist;
    float my = c0.y_mm + a * dy / dist;
    float px = -dy / dist;
    float py = dx / dist;

    out[0] = point_make(mx + h * px, my + h * py);
    if (h < eps) {
        return 1U;
    }

    out[1] = point_make(mx - h * px, my - h * py);
    return 2U;
}

static leg_point_t solve_d_from_b_c(leg_point_t b, leg_point_t c)
{
    float dx = b.x_mm - c.x_mm;
    float dy = b.y_mm - c.y_mm;
    float len = sqrtf(dx * dx + dy * dy);

    if (len < 1.0e-9f) {
        return c;
    }

    return point_make(c.x_mm + LEG_CD_MM * dx / len,
                      c.y_mm + LEG_CD_MM * dy / len);
}

static float theta1_from_a(leg_side_t side, leg_point_t a, leg_point_t o1)
{
    if (side == LEG_SIDE_LEFT) {
        return rad_to_deg(atan2f(o1.x_mm - a.x_mm, a.y_mm - o1.y_mm));
    }
    return rad_to_deg(atan2f(a.x_mm - o1.x_mm, a.y_mm - o1.y_mm));
}

static float theta2_from_b(leg_side_t side, leg_point_t b, leg_point_t o2)
{
    if (side == LEG_SIDE_LEFT) {
        return rad_to_deg(atan2f(b.y_mm - o2.y_mm, o2.x_mm - b.x_mm));
    }
    return rad_to_deg(atan2f(b.y_mm - o2.y_mm, b.x_mm - o2.x_mm));
}

float leg_kinematics_ft_raw_to_theta1_deg(leg_side_t side, uint16_t raw)
{
    if (side == LEG_SIDE_LEFT) {
        return LEG_THETA1_MAX_DEG -
               (((float)raw - LEG_FT0_RAW_THETA_MAX) * LEG_FT_THETA_SPAN /
                (LEG_FT0_RAW_THETA_MIN - LEG_FT0_RAW_THETA_MAX));
    }

    return LEG_THETA1_MIN_DEG +
           (((float)raw - LEG_FT3_RAW_THETA_MIN) * LEG_FT_THETA_SPAN /
            (LEG_FT3_RAW_THETA_MAX - LEG_FT3_RAW_THETA_MIN));
}

uint16_t leg_kinematics_theta1_deg_to_ft_raw(leg_side_t side, float theta1_deg)
{
    theta1_deg = clampf_local(theta1_deg, LEG_THETA1_MIN_DEG, LEG_THETA1_MAX_DEG);

    if (side == LEG_SIDE_LEFT) {
        float raw = LEG_FT0_RAW_THETA_MAX +
                    ((LEG_THETA1_MAX_DEG - theta1_deg) *
                     (LEG_FT0_RAW_THETA_MIN - LEG_FT0_RAW_THETA_MAX) /
                     LEG_FT_THETA_SPAN);
        return round_to_u16(raw, LEG_FT0_RAW_THETA_MAX, LEG_FT0_RAW_THETA_MIN);
    }

    float raw = LEG_FT3_RAW_THETA_MIN +
                ((theta1_deg - LEG_THETA1_MIN_DEG) *
                 (LEG_FT3_RAW_THETA_MAX - LEG_FT3_RAW_THETA_MIN) /
                 LEG_FT_THETA_SPAN);
    return round_to_u16(raw, LEG_FT3_RAW_THETA_MIN, LEG_FT3_RAW_THETA_MAX);
}

float leg_kinematics_dm_rad_to_theta2_deg(leg_side_t side, float dm_rad)
{
    float normalized;

    if (side == LEG_SIDE_LEFT) {
        normalized = (LEG_DM_RAD_MAX - dm_rad) / LEG_DM_RAD_SPAN;
    } else {
        normalized = (dm_rad - LEG_DM_RAD_MIN) / LEG_DM_RAD_SPAN;
    }

    return LEG_THETA2_MIN_DEG + normalized * LEG_THETA2_SPAN_DEG;
}

float leg_kinematics_theta2_deg_to_dm_rad(leg_side_t side, float theta2_deg)
{
    float normalized = (theta2_deg - LEG_THETA2_MIN_DEG) / LEG_THETA2_SPAN_DEG;
    normalized = clampf_local(normalized, 0.0f, 1.0f);

    if (side == LEG_SIDE_LEFT) {
        return LEG_DM_RAD_MAX - normalized * LEG_DM_RAD_SPAN;
    }

    return LEG_DM_RAD_MIN + normalized * LEG_DM_RAD_SPAN;
}

leg_kinematics_status_t leg_kinematics_forward_from_angles(leg_side_t side,
                                                           const leg_joint_angles_t *joint,
                                                           leg_kinematics_result_t *result)
{
    if ((joint == 0) || (result == 0) || !leg_side_is_valid(side)) {
        return LEG_KINEMATICS_BAD_ARGUMENT;
    }

    leg_point_t o1;
    leg_point_t o2;
    leg_get_origins(side, &o1, &o2);

    float t1 = deg_to_rad(joint->theta1_deg);
    float t2 = deg_to_rad(joint->theta2_deg);

    leg_point_t a;
    leg_point_t b;

    if (side == LEG_SIDE_LEFT) {
        a = point_make(o1.x_mm - LEG_L1_MM * sinf(t1),
                       o1.y_mm + LEG_L1_MM * cosf(t1));
        b = point_make(o2.x_mm - LEG_L2_MM * cosf(t2),
                       o2.y_mm + LEG_L2_MM * sinf(t2));
    } else {
        a = point_make(o1.x_mm + LEG_L1_MM * sinf(t1),
                       o1.y_mm + LEG_L1_MM * cosf(t1));
        b = point_make(o2.x_mm + LEG_L2_MM * cosf(t2),
                       o2.y_mm + LEG_L2_MM * sinf(t2));
    }

    leg_point_t c_candidates[2];
    uint8_t c_count = circle_intersections(a, LEG_L3_MM, b, LEG_L4_MM, c_candidates);

    result->joint = *joint;
    result->foot_d = point_make(0.0f, 0.0f);

    if (c_count == 0U) {
        return LEG_KINEMATICS_UNREACHABLE;
    }

    leg_candidate_t best;
    uint8_t best_valid = 0U;

    for (uint8_t i = 0; i < c_count; i++) {
        leg_candidate_t candidate;
        candidate.a = a;
        candidate.b = b;
        candidate.c = c_candidates[i];
        candidate.d = solve_d_from_b_c(b, c_candidates[i]);
        candidate.joint = *joint;
        candidate.cross_count = leg_cross_count(o1, a, o2, b, candidate.c, candidate.d);
        candidate.limit_count = angle_limit_count(*joint);
        candidate.limit_penalty = angle_limit_penalty(*joint);

        if ((best_valid == 0U) || candidate_is_better(&candidate, &best)) {
            best = candidate;
            best_valid = 1U;
        }
    }

    result->foot_d = best.d;

    if (best.limit_count > 0U) {
        return LEG_KINEMATICS_LIMIT;
    }

    return LEG_KINEMATICS_OK;
}

leg_kinematics_status_t leg_kinematics_inverse_to_angles(leg_side_t side,
                                                         const leg_point_t *foot_d,
                                                         leg_kinematics_result_t *result)
{
    if ((foot_d == 0) || (result == 0) || !leg_side_is_valid(side)) {
        return LEG_KINEMATICS_BAD_ARGUMENT;
    }

    leg_point_t o1;
    leg_point_t o2;
    leg_get_origins(side, &o1, &o2);

    leg_point_t b_candidates[2];
    uint8_t b_count = circle_intersections(o2, LEG_L2_MM, *foot_d, LEG_BD_MM, b_candidates);

    result->foot_d = *foot_d;
    result->joint.theta1_deg = 0.0f;
    result->joint.theta2_deg = 0.0f;

    if (b_count == 0U) {
        return LEG_KINEMATICS_UNREACHABLE;
    }

    leg_candidate_t best;
    uint8_t best_valid = 0U;

    for (uint8_t bi = 0; bi < b_count; bi++) {
        leg_point_t b = b_candidates[bi];
        float d_bd = point_distance(b, *foot_d);

        if (d_bd < 1.0e-9f) {
            continue;
        }

        leg_point_t c = point_make(b.x_mm - LEG_L4_MM * (foot_d->x_mm - b.x_mm) / d_bd,
                                   b.y_mm - LEG_L4_MM * (foot_d->y_mm - b.y_mm) / d_bd);

        leg_point_t a_candidates[2];
        uint8_t a_count = circle_intersections(o1, LEG_L1_MM, c, LEG_L3_MM, a_candidates);

        for (uint8_t ai = 0; ai < a_count; ai++) {
            leg_candidate_t candidate;
            candidate.a = a_candidates[ai];
            candidate.b = b;
            candidate.c = c;
            candidate.d = *foot_d;
            candidate.joint.theta1_deg = theta1_from_a(side, candidate.a, o1);
            candidate.joint.theta2_deg = theta2_from_b(side, candidate.b, o2);
            candidate.cross_count = leg_cross_count(o1, candidate.a, o2, b, c, *foot_d);
            candidate.limit_count = angle_limit_count(candidate.joint);
            candidate.limit_penalty = angle_limit_penalty(candidate.joint);

            if ((best_valid == 0U) || candidate_is_better(&candidate, &best)) {
                best = candidate;
                best_valid = 1U;
            }
        }
    }

    if (best_valid == 0U) {
        return LEG_KINEMATICS_UNREACHABLE;
    }

    result->foot_d = best.d;
    result->joint = best.joint;

    if (best.limit_count > 0U) {
        return LEG_KINEMATICS_LIMIT;
    }

    return LEG_KINEMATICS_OK;
}

leg_kinematics_status_t leg_kinematics_forward_from_actuator(leg_side_t side,
                                                             const leg_actuator_state_t *actuator,
                                                             leg_kinematics_result_t *result)
{
    if ((actuator == 0) || (result == 0) || !leg_side_is_valid(side)) {
        return LEG_KINEMATICS_BAD_ARGUMENT;
    }

    leg_joint_angles_t joint;
    joint.theta1_deg = leg_kinematics_ft_raw_to_theta1_deg(side, actuator->ft_raw);
    joint.theta2_deg = leg_kinematics_dm_rad_to_theta2_deg(side, actuator->dm_rad);

    return leg_kinematics_forward_from_angles(side, &joint, result);
}

leg_kinematics_status_t leg_kinematics_fixed_ft_forward(leg_side_t side,
                                                        uint16_t fixed_ft_raw,
                                                        float dm_rad,
                                                        leg_kinematics_result_t *result)
{
    leg_actuator_state_t actuator;
    actuator.ft_raw = fixed_ft_raw;
    actuator.dm_rad = dm_rad;
    return leg_kinematics_forward_from_actuator(side, &actuator, result);
}

leg_kinematics_status_t leg_kinematics_inverse_to_actuator(leg_side_t side,
                                                           const leg_point_t *foot_d,
                                                           leg_actuator_state_t *actuator,
                                                           leg_kinematics_result_t *result)
{
    if ((foot_d == 0) || (actuator == 0) || (result == 0) || !leg_side_is_valid(side)) {
        return LEG_KINEMATICS_BAD_ARGUMENT;
    }

    leg_kinematics_status_t status = leg_kinematics_inverse_to_angles(side, foot_d, result);
    if (status != LEG_KINEMATICS_OK) {
        return status;
    }

    actuator->ft_raw = leg_kinematics_theta1_deg_to_ft_raw(side, result->joint.theta1_deg);
    actuator->dm_rad = leg_kinematics_theta2_deg_to_dm_rad(side, result->joint.theta2_deg);
    return LEG_KINEMATICS_OK;
}
