package frc.robot.allcommands;

public class AllCommandsConstants {
    public static class ShootToSpeaker {
        public final static double SPEED_RELEASE = 0.6;
    }

    public static class ScoreAmp {
        public static final double UPPER_ROLLS_SPEED_RPS = 0;
        public static final double LOWER_ROLLS_SPEES_RPS = 0;
        public static final double SPEED_RELEASE = 0.7;
    }

    public static class CollectFromSource {
        public static final double UPPER_ROLLS_SPEED_RPS = -3;
        public static final double LOWER_ROLLS_SPEES_RPS = -3;
        public static final double BRING_BACK_NOTE_TO_SHOOTER_LOADER_SPEED = 0.2;
        public static final double PITCHER_DEGREES = 40;
        public static final double LOADER_SPEED_TO_INSIDE = 0.5;
    }

    public static class OpenIntake {
        public static final double COLLECTING_WRIST_ANGLE_DEGREE = -27;
        public static final double GRIPPER_COLLECTING_SPEED = 0.5;
        public static final double START_GRIPPER_WRIST_ANGLE_DEGREE = 0;
    }

    public static class Close {
        public static final double CLOSED_WRIST_ANGLE_DEGREE = 180;
        public static final double STOP_GRIPPER_WRIST_ANGLE_DEGREE = 160;
    }

    public static class Handoff {
        public static final double WRIST_HANDOFF_ANGLE_DEGRRES = 194.8;
        public static final double PITCHER_DEGRRES = 10;
        public static final double GRIPPER_HANDOFF_SPEED = -0.3;
        public static final double LOADER_HANDOFF_PRECENTAGE_OUTPUT = 0.3;
        public static final double WRIST_STARTING_LOADER_ANGLE_DEGRRE = 170;
    }

    public static class ReadyToShootToSpeaker {
        public static final double UPPER_ROLLERS_SPEED = 40;
        public static final double LOWER_ROLLERS_SPEED = 40;
        public static final double PITCHER_DEGREES = 40;
        public static final double GRIPPER_SPEED = -0.4;

    }

    public static class ReadyToShootToAmp {
        public static final double UPPER_ROLLERS_SPEED = 7.5;
        public static final double LOWER_ROLLERS_SPEED = 12.5;
        public static final double PITCHER_DEGREES = 42;
        public static final double GRIPPER_SPEED = -0.5;
    }
}