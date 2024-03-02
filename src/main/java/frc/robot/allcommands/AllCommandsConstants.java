package frc.robot.allcommands;

public class AllCommandsConstants {
    public static class ScoreAmp {
        public static final double UPPER_ROLLS_SPEED_RPS = -20;
        public static final double LOWER_ROLLS_SPEES_RPS = 80;
        public static final double SPEED_RELEASE = 1;
        public static final double AMP_DEGREES = 60;
    }

    public static class GetReadyToScoreAMP {
        public static final double AMP_DEGREES = 60;
    }

    public static class OpenIntake {
        public static final double COLLECTING_WRIST_ANGLE_DEGREE = -40;
        public static final double UPPER_GRIPPER_COLLECTING_SPEED = 0.5;
        public static final double LOWER_GRIPPER_COLLECTING_SPEED = -0.5;
        public static final double START_GRIPPER_WRIST_ANGLE_DEGREE = 0;
    }

    public static class Close {
        public static final double CLOSED_WRIST_ANGLE_DEGREE = 155;
        public static final double STOP_GRIPPER_WRIST_ANGLE_DEGREE = 155;
    }
}