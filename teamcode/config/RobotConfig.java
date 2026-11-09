package org.firstinspires.ftc.teamcode.config;

public final class RobotConfig {
    private RobotConfig() {}

    public static final String FR_NAME = "FR";
    public static final String BR_NAME = "BR";
    public static final String FL_NAME = "FL";
    public static final String BL_NAME = "BL";
    public static final String IMU_NAME = "imu";
    public static final String INTAKE_NAME      = "Intake";
    public static final String SHOOTER_NAME     = "ShooterRight";
    public static final String SHOOTER2_NAME    = "ShooterLeft";
    public static final String FEEDER_NAME      = "Feeder";
    public static final String LEFT_SERVO_NAME  = "LeftServo";
    public static final String RIGHT_SERVO_NAME = "RightServo";
    public static final String LIMELIGHT_NAME   = "limelight-dolphin";
    public static final int    LIMELIGHT_PIPELINE = 0;

    public static final double WHEEL_BASE_M   = 0.35;
    public static final double TRACK_WIDTH_M  = 0.38;
    public static final double L = WHEEL_BASE_M / 2.0;
    public static final double W = TRACK_WIDTH_M / 2.0;
    public static final double K = L + W;
    public static final double WHEEL_RADIUS_M = 0.0375;
    public static final double TICKS_PER_REV  = 560.0;
    public static final double TICKS_PER_METER = TICKS_PER_REV / (2.0 * Math.PI * WHEEL_RADIUS_M);
    public static final double MAX_LINEAR_VELOCITY_MPS = 1.2;
    public static final double ACCEL_LIMIT_MPS2        = 3.0;
    public static final double DRIVE_P = 10.0;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;
    public static final double DRIVE_F = 12.0;

    public static final double SHOOTER_TARGET_RPM     = 145.0;
    public static final double SHOOTER_FIRE_RPM       = 137.0;
    public static final double SHOOTER_FIRE_RPM_LOW   = 130.0;
    public static final double SHOOTER_kP             = 50.0;
    public static final double SHOOTER_kI             = 0.0;
    public static final double SHOOTER_kD             = 0.0;
    public static final double SHOOTER_kF             = 14.0;
    public static final double SHOOTER_TICKS_FALLBACK = 560.0;
    public static final double SHOOTER_SPINDOWN_S     = 0.5;
    public static final double SHOOTER_BLOCKING_FEED_DURATION_S = 2.5;
    public static final double SHOOTER_SPINUP_TIMEOUT_S = 3.0;

    public static final double FEEDER_MAX_POWER   = 0.9;
    public static final double INTAKE_MAX_POWER   = 0.9;
    public static final double INTAKE_FIRE_POWER  = 0.8;
    public static final double INTAKE_RAMP_RATE   = 1.0;
    public static final double LEFT_SERVO_DEFAULT  = 0.65;
    public static final double LEFT_SERVO_ACTIVE   = 0.25;
    public static final double RIGHT_SERVO_DEFAULT = 0.70;
    public static final double RIGHT_SERVO_ACTIVE  = 0.30;
    public static final double TRIGGER_THRESH = 0.1;

    public static final double FMAP_TO_FIELD_TX    = 1.8288;
    public static final double FMAP_TO_FIELD_TY    = 1.7835;
    public static final double FMAP_TO_FIELD_THETA = Math.PI;
    public static final boolean FLIP_FIELD_Y       = true;
    public static final double HIVE_PIVOT_HEIGHT_M = 1.075;
    public static final double MAX_HIVE_TILT_DEG   = 35.0;
    public static final double PITCH_CHECK_TOL_DEG = 8.0;
    public static final boolean FLIP_VISION_TH_SIGN = false;
    public static final double VISION_TH_OFFSET    = 0.0;
    public static final double VISION_CONDITIONAL_DEFAULT_TIMEOUT_S = 0.2;

    // === Y mirror fix (12ft field) ===
    public static final double FIELD_LENGTH_M = 3.6576;
    public static final boolean ENABLE_Y_FLIP_FIX = true;

    // Hysteresis margin: prefer current state unless the other candidate
    // wins by at least this much. Prevents flip-flop near midpoint.
    public static final double Y_FLIP_HYSTERESIS_M = 0.10;

    // Reference weighting
    public static final double Y_FLIP_REF_WEIGHT_TRAJ  = 1.0;  // Auto: path target Y
    public static final double Y_FLIP_REF_WEIGHT_EKF   = 1.0;  // Auto: EKF Y
    public static final double Y_FLIP_REF_WEIGHT_PREV  = 1.0;  // both: last accepted
    public static final double Y_FLIP_PREV_DECAY_MS    = 1000.0; // prev weight decay when stale

    // === Outlier gate (disabled as requested) ===
    public static final boolean ENABLE_VISION_OUTLIER_GATE = false;
    public static final double MAX_VISION_JUMP_M = 0.60;

    // === EKF ===
    public static final boolean FUSE_VISION_THETA    = false;
    public static final double VISION_XY_DEADBAND_M  = 0.02;
    public static final double VISION_EMA_ALPHA      = 0.30;
    public static final double VISION_TRUST_FACTOR   = 0.8;
    public static final long   MAX_VISION_STALENESS_MS = 300;
    public static final int    MIN_TAGS_FOR_UPDATE   = 2;
    public static final double VISION_LINEAR_NOISE   = 0.05;
    public static final double VISION_ANGULAR_NOISE  = 0.08;
    public static final double ODOMETRY_LINEAR_NOISE = 0.05;
    public static final double ODOMETRY_ANGULAR_NOISE = 0.03;

    public static final double KP_X     = 5.0;
    public static final double KP_Y     = 5.0;
    public static final double KP_THETA = 3.5;
}