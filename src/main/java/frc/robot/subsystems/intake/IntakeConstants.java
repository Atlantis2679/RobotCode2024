package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class IntakeConstants {
    // Motor and Encoder ID
    public static final int CAN_SPARK_MAX_INTAKE_ID = 0;
    public static final int CAN_SPARK_MAX_INTAKE_PIVOT_ID = 0;
    public static final int DUTY_CYCLE_ENCODER = 0;

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
    public static final double MAX_ACCELERATION_JOINT = 0;
    public static final double MAX_VELOCITY_JOINT = 0;

    //SlewRateLimiter 
    public static final double INTAKE_LIMIT_ACCELERATION_VOLTEG_PER_SECOND = 12;
    public static final double JOINT_LIMIT_ACCELERATION_VOLTEG_PER_SECOND = 12;

}
