package frc.robot.subsystems.swerve;

import edu.wpi.first.math.util.Units;

public class SwerveContants {
    public final static double GEAR_RATIO_DRIVE = 6.75;
    public final static double GEAR_RATIO_ANGLE = 12.8;
    public final static double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);

    public final static double MAX_SPEED_MPS = 5;
    public final static double MAX_VOLTAGE = 11.5;
    public final static double MAX_ANGULAR_VELOCITY = 4.5;
    public final static double MAX_MODULE_SPEED = 4.5;
    
    public final static double TRACK_WIDTH_M = 0.575;
    public final static double TRACK_LENGTH_M = 0.575;
    public final static double TRACK_RADIUS_M = 0.407;

    public final static double MODULE_BL_ABSOLUTE_ANGLE_OFFSET_DEGREES = -60.73242;
    public final static double MODULE_FL_ABSOLUTE_ANGLE_OFFSET_DEGREES = -13.5;
    public final static double MODULE_FR_ABSOLUTE_ANGLE_OFFSET_DEGREES = -56.25 + 180;
    public final static double MODULE_BR_ABSOLUTE_ANGLE_OFFSET_DEGREES = -22.93945;

    public final static double MODULE_ANGLE_KP = 1.8 * 12;
    public final static double MODULE_ANGLE_KI = 0;
    public final static double MODULE_ANGLE_KD = 0;

    public final static double SENSETIVE_FORWARD_MULTIPLIER = 0.3;
    public final static double SENSETIVE_ROTATION_MULTIPLIER = 0.3;

    public class PathPlanner {
        public final static double TRANSLATION_KP = 1;
        public final static double TRANSLATION_KI = 0;
        public final static double TRANSLATION_KD = 0.027;

        public final static double ROTATION_KP = 0.1;
        public final static double ROTATION_KI = 0;
        public final static double ROTATION_KD = 0.001;
    }
}
