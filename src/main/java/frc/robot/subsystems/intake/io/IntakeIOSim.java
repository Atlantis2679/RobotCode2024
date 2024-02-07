package frc.robot.subsystems.intake.io;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.logfields.LogFieldsTable;

public class IntakeIOSim extends IntakeIO {
    private final FlywheelSim rollersMotor = new FlywheelSim(
            DCMotor.getNeo550(1),
            ROLLERS_GEAR_RATIO,
            ROLLERS_JKG_METERS_SQUARED);

    private final SingleJointedArmSim wristMotor = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            JOINT_GEAR_RATIO,
            WRIST_JKG_METERS_SQUARED,
            0.35,
            -Math.PI,
            Math.PI,
            true,
            1);

    @Override
    protected void periodicBeforeFields() {
        rollersMotor.update(0.02);
        wristMotor.update(0.02);
    }

    public IntakeIOSim(LogFieldsTable logFieldsTable) {
        super(logFieldsTable);
    }

    @Override
    public void setRollerSpeedPrecentOutput(double rollersSpeed) {
        rollersMotor.setInputVoltage(rollersSpeed * 12);
    }

    @Override
    public void setWristVoltage(double voltage) {
        wristMotor.setInputVoltage(voltage);
    }

    @Override
    protected double getWristAngleDegrees() {
        return Math.toDegrees(wristMotor.getAngleRads() + 2);
    }

    @Override
    protected boolean getNoteDetectorValue() {
        return false;
    }
}
