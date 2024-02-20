package frc.robot.subsystems.flywheel;

public class FlywheelConstants {
    public static final double AMT103_PULSES_PER_ROUND = 2048;
    public static final double UPPER_ROLLER_GEAR_RATIO = 5;
    public static final double LOWER_ROLLER_GEAR_RATIO = 5;
    public static final double MAX_VOLTAGE = 11;
    public static final double MAX_ACCELERATION_VOLTS_PER_SECOND = 12;
    public static final double SPEED_TOLERANCE_RPS = 2;

    public final class RotateFlywheel {
        public static final double KP = 0;
        public static final double KI = 0;
        public static final double KD = 0;

        public static final double KS = 0.04;
        public static final double KV = 6;
        public static final double KA = 0;
    }
}
