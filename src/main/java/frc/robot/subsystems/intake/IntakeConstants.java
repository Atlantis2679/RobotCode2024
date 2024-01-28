package frc.robot.subsystems.intake;


public class IntakeConstants {
    // Motor and Encoder ID
    public static final int CAN_SPARK_MAX_ROLLERS_ID = 0;
    public static final int CAN_SPARK_MAX_WRIST_ID = 0;
    public static final int DUTY_CYCLE_ENCODER_WRIST = 0;

    // Angle PID
    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;

    // set
    public static final double SCORING_SPEED = 1;
    public static final double INTAKE_SPEED = -1;

    // Feedforward Intake arm
    public static final double KS = 0;
    public static final double KG = 0;
    public static final double KV = 0;
    public static final double KA = 0;

    // trapezoidProfile
    public static final double WRIST_MAX_ACCELERATION = 0;
    public static final double MAX_VELOCITY_JOINT = 0;

    //SlewRateLimiter 
    public static final double ROLLERS_LIMIT_ACCELERATION_VOLTEG_PER_SECOND = 12;
    public static final double WRIST_LIMIT_ACCELERATION_VOLTEG_PER_SECOND = 12;

    //Intake Controller
    public static final double INTAKE_CONTROLLER_MULTIPLIER = 12;

    //Intake button 
    public static final double STOP_INTAKE_MOTOR = 0;

    //IntakeAngle
    public static final double STOP_INTAKE_ANGLE_MOTOR = 0;

    //command combination 
    public static final double COLLECTING_POSITION_DEGREE = 0;
    public static final double COLLECTING_POSITION_INTAKE_SPEED = 0;

    //sim
    public static final double GEARING_JOINT = 60;
    public static final double GEARING_INTAKE = 10;


}
