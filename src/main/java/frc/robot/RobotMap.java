package frc.robot;

import edu.wpi.first.wpilibj.SPI;

public class RobotMap {

    public final static SPI.Port NAVX_PORT = SPI.Port.kMXP;

    public class Controllers {
        public static final int DRIVER_PORT = 0;
        public static final int OPERTATOR_PORT = 1;
    }

    public class Module0 {
        public final static int DRIVE_MOTOR_ID = 20;
        public final static int ANGLE_MOTOR_ID = 21;
        public final static int ENCODER_ID = 50;
    }

    public class Module1 {
        public final static int DRIVE_MOTOR_ID = 22;
        public final static int ANGLE_MOTOR_ID = 23;
        public final static int ENCODER_ID = 51;
    }

    public class Module2 {
        public final static int DRIVE_MOTOR_ID = 24;
        public final static int ANGLE_MOTOR_ID = 25;
        public final static int ENCODER_ID = 52;
    }

    public class Module3 {
        public final static int DRIVE_MOTOR_ID = 26;
        public final static int ANGLE_MOTOR_ID = 27;
        public final static int ENCODER_ID = 53;
    }

    public class Intake {
        public static final int ROLLERS_MOTOR_ID = 13;
        public static final int WRIST_MOTOR_ID = 8;
        public static final int BEAM_BREAK_ID = 0;
    }
  
    public class Flywheel {
        public final static int UPPER_ROLLER_MOTOR_ID = 0;
        public final static int LOWER_ROLLER_MOTOR_ID = 0;
        public final static int UPPER_ROLLER_ENCODER_CHANNEL_A = 0;
        public final static int UPPER_ROLLER_ENCODER_CHANNEL_B = 0;
        public final static int LOWER_ROLLER_ENCODER_CHANNEL_A = 0;
        public final static int LOWER_ROLLER_ENCODER_CHANNEL_B = 0;
    }

    public class Pitcher {
        public final static int MOTOR_ID = 0;
        public final static int ENCODER_ID = 0;
    }

    public class Loader {
        public final static int MOTOR_ID = 0;
    }
}
