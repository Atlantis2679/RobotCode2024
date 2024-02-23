package frc.robot.allcommands;

public class AllCommandsConstants {

    public static class ShootToSpeaker {
        public final static double SPEED_RELEASE = 0.6;
    }

    public static class ScoreAmp {
        public final static double UPPER_ROLLS_SPEED_RPS = 0;
        public final static double LOWER_ROLLS_SPEES_RPS = 0;
        public static final double SPEED_RELEASE = 0.4;
    }

    public static class CollectFromSource {
        public final static double UPPER_ROLLS_SPEED_RPS = -3;
        public final static double LOWER_ROLLS_SPEES_RPS = -3;
        public final static double BRING_BACK_NOTE_TO_SHOOTER = 0.2;
        public final static double DEGREES_SOURCE = 0;
    }

    public static class Open {
        public static final double COLLECTING_WRIST_ANGLE_DEGREE = -27;
        public static final double GRIPPER_COLLECTING_SPEED = 0.5;
        public static final double START_GRIPPER_WRIST_ANGLE_DEGREE = 0;
    }

    public static class Close {
        public static final double CLOSED_WRIST_ANGLE_DEGREE = 180;
        public static final double STOP_GRIPPER_WRIST_ANGLE_DEGREE = 160;
    }

    public static class HandOff {
        public static final double WRIST_HANDOFF_ANGLE_DEGRRES = 194.8;
        public static final double PITCHER_HANDOF_ANGLE_DEGRRES = -20;
        public static final double GRIPPER_HANDOFF_SPEED = -0.3;
        public static final double LOADER_HANDOFF_PRECENTAGE_OUTPUT = 0.3;
        public static final double WRIST_STARTING_LOADER_ANGLE_DEGRRE = 190;
    }

    public static class ReadyToShootToSpeaker {
        public static final double UPPER_ROLLERS_SPEED = 6;
        public static final double LOWER_ROLLERS_SPEED = 6;

    }

    public static class ReadyToScoreAmp {
        public static final double UPPER_ROLLERS_SPEED = 0;
        public static final double LOWEER_ROLLERS_SPEED = 0;

    }

}