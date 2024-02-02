package frc.robot.subsystems.intake.io;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.logfields.LogFieldsTable;

public class IntakeIOsim extends IntakeIO{
    private final FlywheelSim rollersMotor;
    private final SingleJointedArmSim wristMotor;
    
    DigitalInput beamBreak = new DigitalInput(BEAM_BREAK_ID);


    public IntakeIOsim(LogFieldsTable logFieldsTable){
        super(logFieldsTable);
        rollersMotor = new FlywheelSim(
        DCMotor.getNeo550(CAN_SPARK_MAX_ROLLERS_ID),  
        ROLLERS_GEARING,  ROLLERS_JKG_METERS_SQUARED);

        wristMotor = new SingleJointedArmSim(
            DCMotor.getNEO(CAN_SPARK_MAX_WRIST_ID),
             JOINT_GEARING, WRIST_JKG_METERS_SQUARED,
              0.35, 
            -Math.PI, Math.PI,
            true,
            1);
    }


    @Override
    protected double getWristSpeed() {
        return wristMotor.getVelocityRadPerSec();
    }

    @Override
    public void setRollerSpeedPrecentOutput(double rollersSpeed) {
        rollersMotor.setInput(rollersSpeed);
    }


    @Override
    public void setWristSpeedPrecentOutput(double wristSpeed) {
        wristMotor.setInput(wristSpeed);
    }



    @Override
    protected double getWristAngleDegrees() {
        return Math.toDegrees(wristMotor.getAngleRads());
    }



    @Override
    protected boolean getBeamBreakValue() {
        return false;
    }


}
