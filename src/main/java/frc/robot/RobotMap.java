package frc.robot;

import edu.wpi.first.wpilibj.SPI;

public class RobotMap {
    public final static SPI.Port NAVX_PORT = SPI.Port.kMXP;
    public static final String FRONT_CAMERA_NAME = "FrontCam";

    public class Controllers {
        public static final int DRIVER_PORT = 0;
        public static final int OPERTATOR_PORT = 1;
    }

    public class ModuleFL {
        public final static int DRIVE_MOTOR_ID = 20;
        public final static int ANGLE_MOTOR_ID = 21;
        public final static int ENCODER_ID = 50;
    }

    public class ModuleFR {
        public final static int DRIVE_MOTOR_ID = 26;
        public final static int ANGLE_MOTOR_ID = 27;
        public final static int ENCODER_ID = 53;
    }

    public class ModuleBL {
        public final static int DRIVE_MOTOR_ID = 24;
        public final static int ANGLE_MOTOR_ID = 25;
        public final static int ENCODER_ID = 52;
    }

    public class ModuleBR {
        public final static int DRIVE_MOTOR_ID = 22;
        public final static int ANGLE_MOTOR_ID = 23;
        public final static int ENCODER_ID = 51;
    }

    public class Intake {
        public static final int LOWER_ROLLER_MOTOR_ID = 11;
        public static final int UPPER_ROLLER_MOTOR_ID = 14;
        public static final int WRIST_MOTOR_ID = 10;
        public static final int WRIST_ENCODER_ID = 0;
        public static final int BEAM_BREAK_ID = 1;
    }

    public class Elevator {
        public final static int ELEVATOR_MOTOR_RIGHT_ID = 12;
        public final static int ELEVATOR_MOTOR_LETF_ID = 13;
        public final static int LEFT_LIMIT_SWITCH_ID = 2;
        public final static int RIGHT_LIMIT_SWITCH_ID = 3;

    }
}
