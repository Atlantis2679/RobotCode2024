package frc.robot.subsystems.wrist;

public class WristConstants {

    public class MoveToAngle {
        public static final boolean USE_CLOSED_LOOP_PROFILE = true;
    }

    // Angle PID
    public static final double KP = 0.1; // 0.1
    public static final double KI = 0;
    public static final double KD = 0;

    // Feedforward wrist
    public static final double KS = 0;
    public static final double KG = -0.5; // 0.556000
    public static final double KV = 0.023;
    public static final double KA = 0;

    public class Sim {
        public static final double KS = 0;
        public static final double KG = -0.5; // 0.556000
        public static final double KV = 0.023;
        public static final double KA = 0;

    }
    // set

    public class IsAtAngle {
        public static final double MAX_WRIST_ANGLE_DEVAITION = 10;
        public static final double MIN_WRIST_ANGLE_DEVAITION = -10;
        
    }
    public static final double WRIST_VOLTAGE_LIMIT = 4;

    // trapezoidProfile
    public static final double WRIST_MAX_ACCELERATION_DEG_PER_SEC = 200;
    public static final double WRIST_MAX_VELOCITY_DEG_PER_SEC = 200;

    // SlewRateLimiter

    // Intake button
    public static final double STOP_INTAKE_MOTOR = 0;

    // IntakeAngle
    public static final double STOP_INTAKE_ANGLE_MOTOR = 0;

    // wrist angles
    public static final double WIRST_ANGLE_UPPER_BOUND_DEGREES = 250;
    public static final double WIRST_ANGLE_OFFSET_DEGREES = 220 ;

    // rollers speeds

    // sim
    public static final double JOINT_GEAR_RATIO = 60;
    public static final double ROLLERS_GEAR_RATIO = 10;
    public static final double ROLLERS_JKG_METERS_SQUARED = 0.05;
    public static final double WRIST_JKG_METERS_SQUARED = 0.172515;
    public static final int ENCODER_SIM_ID = 2;

    public static final double WRIST_TURNING_MAX_DEGREES = 210;
    public static final double WRIST_TURNING_MIN_DEGREES = -10;

    public class ManualController {
        public static final double SPEED_MULTIPLIER = 4;
    }

}
