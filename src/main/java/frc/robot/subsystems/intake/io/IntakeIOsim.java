package frc.robot.subsystems.intake.io;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.logfields.LogFieldsTable;

public class IntakeIOsim extends IntakeIO{
    private final FlywheelSim intakeFlywheelSim;
    private final SingleJointedArmSim wristMotor;
    
    DigitalInput beamBreak = new DigitalInput(BEAM_BREAK_ID);


    public IntakeIOsim(LogFieldsTable logFieldsTable){
        super(logFieldsTable);
        intakeFlywheelSim = new FlywheelSim(DCMotor.getNeo550(CAN_SPARK_MAX_ROLLERS_ID),  ROLLERS_GEARING,  ROLLERS_INERTIA);
        wristMotor = new SingleJointedArmSim(
            DCMotor.getNEO(CAN_SPARK_MAX_WRIST_ID),
             JOINT_GEARING, WRIST_INERTIA,
              1,
                -Math.PI, Math.PI,
                 true,
                  1);
    }


        
    @Override
    protected double getRollersSpeed() {
     return intakeFlywheelSim.
    }


    @Override
    protected double getWristSpeed() {
        return wristMotor.getVelocityRadPerSec();
    }


    @Override
    protected boolean getBeamBreakValue() {
        
    }


    @Override
    public void setRollerSpeed(double rollersSpeed) {

    }


    @Override
    public void setWristSpeed(double wristSpeed) {
    }


}
