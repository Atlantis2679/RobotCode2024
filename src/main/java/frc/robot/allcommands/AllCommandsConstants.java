package frc.robot.allcommands;

public class AllCommandsConstants {
    public static class ScoreAmp {
        public static final double UPPER_ROLLS_SPEED_RPS = -0.35;
        public static final double LOWER_ROLLS_SPEES_RPS = 0.52;
        public static final double AMP_DEGREES = 68;
    }

    public static class GetReadyToScoreAMP {
        public static final double AMP_DEGREES = 68;
    }

    public static class OpenIntake {
        public static final double COLLECTING_WRIST_ANGLE_DEGREE = -45;
        public static final double UPPER_GRIPPER_COLLECTING_SPEED = 0.5;
        public static final double LOWER_GRIPPER_COLLECTING_SPEED = -0.5;
        public static final double START_GRIPPER_WRIST_ANGLE_DEGREE = 0;
    }

    public static class Close {
        public static final double CLOSED_WRIST_ANGLE_DEGREE = 150;
        public static final double STOP_GRIPPER_WRIST_ANGLE_DEGREE = 150;
    }

    // public static class Deliver {
        
        
    // }
}