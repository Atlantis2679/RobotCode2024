package frc.robot.allcommands;

public class AllCommandsConstants {

package frc.robot.allcommands;

public class AllCommandsConstants {
    public class ShootToSpeaker {
    public final static double SPEED_RELEASE = 0.6;
    }

    public static class ScoreAmp {
        public final static double UPPER_ROLLS_SPEED_RPS = 0;
        public final static double LOWER_ROLLS_SPEES_RPS = 0;
    }

    public static class CollectFromSource {
        public final static double UPPER_ROLLS_SPEED_RPS = 0;
        public final static double LOWER_ROLLS_SPEES_RPS = 0;
        public final static double BRING_BACK_NOTE_TO_SHOOTER = 0;
        public final static double DEGREES_SOURCE = 0;
    }

  
    public static class Open {
        public static final double COLLECTING_WRIST_ANGLE_DEGREE = -27;
        public static final double GRIPPER_COLLECTING_SPEED = 0.5;
        public static final double START_GRIPPER_WRIST_ANGLE_DEGREE = 90;
    }

    public static class Close {
        public static final double CLOSED_WRIST_ANGLE_DEGREE = 180;
        public static final double STOP_ROLLERS_WRIST_ANGLE_DEGREE = 160;
    }
}
