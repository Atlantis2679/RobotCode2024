package frc.robot.subsystems.pitcher;

public class PitcherConstants {
    public static final double MAX_VOLTAGE = 8;
    public static final double ANGLE_OFFSET_DEGREES = 0;
    public static final double GEAR_RATIO = 20;

    public static final double KP = 0.04;
    public static final double KI = 0;
    public static final double KD = 0.04;

    public static final double KS = 0;
    public static final double KG = 0.113077;
    public static final double KV = 0.007000;
    public static final double MAX_VELOCITY_DEG_PER_SEC = 120;
    public static final double MAX_ACCELERATION_DEG_PER_SEC = 200;

    public class AdjustToAngle {
        public static final boolean USE_OPEN_LOOP = false;
    }
}
