/**
 * @file    arm_kinematics.c
 * @brief   机械臂正逆解（FK/IK）实现：纯算法，不依赖硬件
 *
 * 构型：θ1 水平 + θ2/θ3/θ4 共面俯仰（解耦）。
 * 坐标系：基座系 +X 前 / +Y 左 / +Z 上；长度 mm，角度接口用度。
 *
 * 说明：F103 无 FPU，sin/cos/atan2/sqrt 走软浮点，控制周期需实测评估。
 */

#include "arm_kinematics.h"

/* ============================================================
 * 内部小工具
 * ============================================================ */

static float d2r(float d) { return d * (ARM_PI / 180.0f); }
static float r2d(float r) { return r * (180.0f / ARM_PI); }

/* 钳位 */
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ============================================================
 * FK：关节角 → 末端（θ4 轴心）基座系位置
 * ============================================================ */
void arm_fk(const arm_joint_t *j, arm_pos_t *out)
{
    float t1 = d2r(j->th1);
    float t2 = d2r(j->th2);
    float t3 = d2r(j->th3);
    float t4 = d2r(j->th4);

    /* 俯仰平面内的 r,z（θ1 旋转后的投影面）：
     *   θ2 抬升：大臂 L2 在垂直面的分量
     *   θ3 相对大臂：小臂 L3
     * 约定：θ2=0 水平前伸，θ2 增 → 末端抬升 z；θ3 增 → 小臂相对大臂继续向上折。
     * 这里采用「大臂 + 小臂」平面链：
     *   r 方向 = L2*cos(θ2) + L3*cos(θ2+θ3)   (水平投影,覆盖 d0 之后)
     *   z 方向 = L2*sin(θ2) + L3*sin(θ2+θ3)   (垂直投影)
     */
    float r_axis = ARM_L2_MM * cosf(t2) + ARM_L3_MM * cosf(t2 + t3);
    float z_axis = ARM_L2_MM * sinf(t2) + ARM_L3_MM * sinf(t2 + t3);

    /* 夹爪 TCP：从 θ4 轴心沿“小臂+腕”方向的固定偏置延伸（相对小臂杆方向，随 θ4 旋转）
     *   小臂杆方向角 = θ2+θ3；腕角 θ4 相对小臂 → 「前」方向角 = θ2+θ3+θ4。
     *   偏置（前 60 + 上 72，换装后净偏置）：
     *     「前」分量 F = 60mm，沿 θ234 方向；
     *     「上」分量 U = 72mm，垂直「前」（θ234+90°）。
     *   俯仰平面内 TCP 相对 θ4 轴心的偏移：
     *     r_off =  F*cos(θ234) - U*sin(θ234)
     *     z_off =  F*sin(θ234) + U*cos(θ234)
     */
    float t234 = t2 + t3 + t4;
    float fwd = ARM_GRIP_FWD_MM;   /* 前 60 */
    float up  = ARM_GRIP_UP_MM;    /* 上 72 */
    float r_off = fwd * cosf(t234) - up * sinf(t234);
    float z_off = fwd * sinf(t234) + up * cosf(t234);

    /* TCP = θ4 轴心 + 折线偏移 */
    float r_plane_eff = r_axis + r_off;
    float z_plane      = z_axis + z_off;

    /* 转回基座系：θ1 决定 x/y 的方位 */
    float r_base = r_plane_eff + ARM_D0_MM;   /* 加上水平偏置（0=共轴） */
    out->x = r_base * cosf(t1);
    out->y = r_base * sinf(t1);
    out->z = z_plane;   /* 未加 H0（由调用方决定是否加离地高度） */
}

/* ============================================================
 * IK：解析法（共面三连杆）
 * ============================================================ */
int arm_ik(const arm_pos_t *p, uint8_t elbow_up, arm_joint_t *out)
{
    float th1, th2, th3;
    float r, z;
    float D, c3, s3;
    float L2 = ARM_L2_MM;
    float L3 = ARM_L3_MM;

    /* 1. θ1：水平方位角 */
    th1 = atan2f(p->y, p->x);

    /* 2. 降维到俯仰平面：r = 水平距离（去掉 d0），z 直接取。 */
    r = sqrtf(p->x * p->x + p->y * p->y) - ARM_D0_MM;
    z = p->z;

    /* 3. 平面二连杆逆解：cos 定理
     *   D = r² + z² ;  cos(θ3) = (D - L2² - L3²) / (2·L2·L3)
     */
    D = r * r + z * z;
    c3 = (D - L2 * L2 - L3 * L3) / (2.0f * L2 * L3);

    /* 不可达判断 */
    if (c3 > 1.0f)  c3 = 1.0f;
    if (c3 < -1.0f) return -1;   /* 超出工作空间 */

    s3 = sqrtf(1.0f - c3 * c3);
    if (elbow_up) s3 = -s3;   /* 肘上：θ3 取负分支 */

    th3 = r2d(atan2f(s3, c3));

    /* θ2：atan2(z,r) - 大臂对小臂末端夹角的补偿 */
    /* ψ = atan2(L3*sin(θ3), L2 + L3*cos(θ3)) */
    float t3 = d2r(th3);
    float psi = atan2f(L3 * sinf(t3), L2 + L3 * cosf(t3));
    th2 = r2d(atan2f(z, r) - psi);

    /* 4. 输出 */
    out->th1 = r2d(th1);
    out->th2 = th2;
    out->th3 = th3;
    out->th4 = 0.0f;   /* θ4 由固定策略决定，不参与位置求解 */

    return 0;
}

/* ============================================================
 * 增量逆解（视觉伺服）：解析雅可比一步牛顿
 * ============================================================ */
int arm_ik_delta(const arm_pos_t *cur, const arm_pos_t *dxyz,
                 const arm_joint_t *j_cur, arm_joint_t *out_dth)
{
    float t1 = d2r(j_cur->th1);
    float t2 = d2r(j_cur->th2);
    float t3 = d2r(j_cur->th3);
    float L2 = ARM_L2_MM;
    float L3 = ARM_L3_MM;

    float c1 = cosf(t1), s1 = sinf(t1);
    float c23 = cosf(t2 + t3), s23 = sinf(t2 + t3);
    float c2 = cosf(t2), s2 = sinf(t2);

    /* 3x3 雅可比 J = ∂(x,y,z)/∂(θ1,θ2,θ3)
     * 末端位置（θ4 轴心）：
     *   x = (d0 + L2*c2 + L3*c23) * c1
     *   y = (d0 + L2*c2 + L3*c23) * s1
     *   z = L2*s2 + L3*s23
     */
    float rho = ARM_D0_MM + L2 * c2 + L3 * c23;
    float drho_d2 = -L2 * s2 - L3 * s23;   /* ∂rho/∂θ2 */
    float drho_d3 = -L3 * s23;             /* ∂rho/∂θ3 */

    /* J 矩阵（行主序） */
    float J11 = -rho * s1;
    float J12 = drho_d2 * c1;
    float J13 = drho_d3 * c1;

    float J21 = rho * c1;
    float J22 = drho_d2 * s1;
    float J23 = drho_d3 * s1;

    float J31 = 0.0f;
    float J32 = L2 * c2 + L3 * c23;
    float J33 = L3 * c23;

    /* 3x3 解析逆：det */
    float det =
        J11 * (J22 * J33 - J23 * J32)
      - J12 * (J21 * J33 - J23 * J31)
      + J13 * (J21 * J32 - J22 * J31);

    if (det > -1e-9f && det < 1e-9f) {
        return -1;   /* 奇异位形，不可逆 */
    }

    /* 逆矩阵（伴随矩阵 / det）作用到 dxyz */
    float dx = dxyz->x, dy = dxyz->y, dz = dxyz->z;

    /* 伴随矩阵 * 向量：A⁻¹ b = adj(A) b / det */
    /* adj(A) 的第 i 行 = Cof 的第 i 列 */
    float A11 = (J22 * J33 - J23 * J32);
    float A12 = (J13 * J32 - J12 * J33);
    float A13 = (J12 * J23 - J13 * J22);

    float A21 = (J23 * J31 - J21 * J33);
    float A22 = (J11 * J33 - J13 * J31);
    float A23 = (J13 * J21 - J11 * J23);

    float A31 = (J21 * J32 - J22 * J31);
    float A32 = (J12 * J31 - J11 * J32);
    float A33 = (J11 * J22 - J12 * J21);

    float d1 = (A11 * dx + A12 * dy + A13 * dz) / det;
    float d2 = (A21 * dx + A22 * dy + A23 * dz) / det;
    float d3 = (A31 * dx + A32 * dy + A33 * dz) / det;

    out_dth->th1 = r2d(d1);
    out_dth->th2 = r2d(d2);
    out_dth->th3 = r2d(d3);
    out_dth->th4 = 0.0f;

    return 1;
}

/* ============================================================
 * 换算层
 * ============================================================ */

/* 关节号 → 方向符号 + 零位 PWM + 行程 */
/* 零位基准：三关节统一用「收起态 PWM 对应关节角 0」（2026-10-11 定案）。
 *   θ2 → ARM_TH2_PWM_STOW / θ3 → ARM_TH3_PWM_STOW / θ4 → ARM_TH4_PWM_UP（收起态上翻）。
 * 机械零位物理角待实测，但在 FK/IK 换算层里三点统一锚定「收起态=0°」。 */

static uint16_t joint_pwm_ref(uint8_t joint)
{
    switch (joint) {
        case 2: return ARM_TH2_PWM_STOW;
        case 3: return ARM_TH3_PWM_STOW;
        case 4: return ARM_TH4_PWM_UP;    /* 腕：收起态上翻初始（统一收起基准） */
        default: return ARM_SERVO_PWM_CENTER;
    }
}

static int8_t joint_dir_sign(uint8_t joint)
{
    switch (joint) {
        case 2: return ARM_TH2_DIR;
        case 3: return ARM_TH3_DIR;
        case 4: return ARM_TH4_DIR;
        default: return ARM_JOINT_DIR_INC;
    }
}

static uint16_t joint_pwm_min(uint8_t joint)
{
    switch (joint) {
        case 2: return ARM_TH2_PWM_MIN;
        case 3: return ARM_TH3_PWM_MIN;
        case 4: return ARM_TH4_PWM_DOWN;
        default: return ARM_SERVO_PWM_MIN;
    }
}

static uint16_t joint_pwm_max(uint8_t joint)
{
    switch (joint) {
        case 2: return ARM_TH2_PWM_MAX;
        case 3: return ARM_TH3_PWM_MAX;
        case 4: return ARM_TH4_PWM_UP;
        default: return ARM_SERVO_PWM_MAX;
    }
}

/* 机械角 → PWM：
 *   pwm = ref + dir_sign * deg * (2000/270)
 *   dir_sign 由关节方向决定：PWM 增对应机械角增则 +，否则 -
 */
int32_t arm_angle_to_pwm(uint8_t joint, float deg)
{
    uint16_t ref = joint_pwm_ref(joint);
    int8_t sign = joint_dir_sign(joint);
    /* 每度对应 PWM = 2000/270 */
    float pwm = (float)ref + (float)sign * deg * (2000.0f / 270.0f);

    uint16_t mn = joint_pwm_min(joint);
    uint16_t mx = joint_pwm_max(joint);
    pwm = clampf(pwm, (float)mn, (float)mx);
    return (int32_t)(pwm + 0.5f);
}

/* PWM → 机械角（上式反解） */
float arm_pwm_to_angle(uint8_t joint, uint16_t pwm)
{
    uint16_t ref = joint_pwm_ref(joint);
    int8_t sign = joint_dir_sign(joint);
    float deg = ((float)pwm - (float)ref) / (float)sign * (270.0f / 2000.0f);
    return deg;
}

/* θ1 底座角 → 步进脉冲（带符号，正数 = ARM_DIR_CW 方向） */
int32_t arm_theta1_to_pulse(float deg)
{
    return (int32_t)(deg * ARM_STEP_PULSE_PER_DEG);
}

/* ============================================================
 * θ1 转向计算：从当前朝向转到目标相对角
 * ============================================================ */
int arm_theta1_turn(float heading_deg, float target_deg, uint8_t force_ccw,
                    arm_th1_turn_t *out)
{
    float delta;

    if (out == NULL)
    {
        return -1;
    }

    /* 目标 − 当前 = 需转的角度差（带符号） */
    delta = target_deg - heading_deg;

    /* 归一化到 (-180, 180] */
    while (delta >  180.0f) delta -= 360.0f;
    while (delta <= -180.0f) delta += 360.0f;

    if (force_ccw)
    {
        /* 强制向右转（顺时针/ARM_DIR_CCW）：
         * 不论 delta 符号，都朝右转；delta 为负(目标在左)时绕一整圈补角。
         * 排爆桶在车体右侧，防机械卡臂。 */
        if (delta < 0.0f)
        {
            delta += 360.0f;   /* 目标在左侧 → 向右绕一圈过去 */
        }
        if (delta < 0.5f)
        {
            out->dir    = ARM_TH1_DIR_CCW;
            out->pulses = 0;
            return 0;
        }
        out->dir    = ARM_TH1_DIR_CCW;
        out->pulses = arm_theta1_to_pulse(delta);
        return 0;
    }

    /* 常规模式：按 delta 符号选方向，短路径 |delta|（已归一化 ≤180）*/
    if (delta > 0.5f)
    {
        out->dir    = ARM_TH1_DIR_CW;      /* 左转/逆时针 */
        out->pulses = arm_theta1_to_pulse(delta);
        return 0;
    }
    else if (delta < -0.5f)
    {
        out->dir    = ARM_TH1_DIR_CCW;     /* 右转/顺时针 */
        out->pulses = arm_theta1_to_pulse(-delta);
        return 0;
    }

    /* |delta|<=0.5° 视作已到位 */
    out->dir    = ARM_TH1_DIR_CW;
    out->pulses = 0;
    return 0;
}
