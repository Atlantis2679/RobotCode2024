package frc.robot.subsystems.intake;

public class IntakeConstants {
    // Motor and Sensors ID

    // Angle PID
        public static final double KP = 0.1;
        public static final double KI = 0;
        public static final double KD = 0;

    // Feedforward wrist
        public static final double KS = 0;
        public static final double KG = 0.161519;
        public static final double KV = 0;
        public static final double KA = 0;

    // set
        public static final double SCORING_SPEED = 1;
        public static final double INTAKE_SPEED = -1;

        public static final double ROLLERS_SPEED_LIMIT_PRECENTAGE = 1;
        public static final double WRIST_VOLTAGE_LIMIT = 12;

    // trapezoidProfile
        public static final double WRIST_MAX_ACCELERATION_DEG_PER_SEC = 90;
        public static final double WRIST_MAX_VELOCITY_DEG_PER_SEC = 90;

    // SlewRateLimiter
        public static final double ROLLERS_ACCELERATION_LIMIT_VOLTAGE_PER_SECOND = 12;

    // Intake button
        public static final double STOP_INTAKE_MOTOR = 0;

    // IntakeAngle
        public static final double STOP_INTAKE_ANGLE_MOTOR = 0;

    // wrist angles

    // rollers speeds
        public static final double COLLECTING_ROLLERS_SPEED = 1;

    // sim
        public static final double JOINT_GEAR_RATIO = 60;
        public static final double ROLLERS_GEAR_RATIO = 10;
        public static final double ROLLERS_JKG_METERS_SQUARED = 0.05;
        public static final double WRIST_JKG_METERS_SQUARED = 0.05;
        public static final int ENCODER_SIM_ID = 2;

        public static final double WRIST_TURNING_MAX_DEGREES = 210;
        public static final double WRIST_TURNING_MIN_DEGREES = -10;

    public class ManualController {
        public static final double SPEED_MULTIPLIER = 11;
    }

    public class MoveToAngle {
        public static final boolean USE_CLOSED_LOOP_PROFILE = true;
    }
    
    public class Open {
        public static final double COLLECTING_WRIST_ANGLE_DEGREE = 210;
    }
    
    public class Close {
        public static final double CLOSED_WRIST_ANGLE_DEGREE = -10;
    }

    public class aimToAmp {
        public static final double AIM_TO_AMP_WRIST_DEGREE = 100;
    }
    
    public class collectFromSource {
        public static final double COLLECT_FROM_SOURCE_WRIST_DEGREE = 100;
        public static final double COLLECT_FROM_SOURCE_ROLLER_SPEED = -1;
    }

}
