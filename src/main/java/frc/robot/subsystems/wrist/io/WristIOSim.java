package frc.robot.subsystems.wrist.io;

import static frc.robot.subsystems.wrist.WristConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.logfields.LogFieldsTable;

public class WristIOSim extends WristIO {
    // private final FlywheelSim rollersMotor = new FlywheelSim(
    //         DCMotor.getNeo550(1),
    //         ROLLERS_GEAR_RATIO,
    //         ROLLERS_JKG_METERS_SQUARED);

    private final SingleJointedArmSim wristMotor = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            JOINT_GEAR_RATIO,
            WRIST_JKG_METERS_SQUARED,
            0.35,
            Math.toRadians(WRIST_TURNING_MIN_DEGREES),
            Math.toRadians(WRIST_TURNING_MAX_DEGREES),
            true,
            1);

    @Override
    protected void periodicBeforeFields() {
        // rollersMotor.update(0.02);
        wristMotor.update(0.02);
    }

    public WristIOSim(LogFieldsTable logFieldsTable) {
        super(logFieldsTable);
    }

    @Override
    public void setWristVoltage(double voltage) {
        Logger.recordOutput("set voltage", voltage);
        wristMotor.setInputVoltage(voltage);
    }

    @Override
    protected double getWristAngleDegrees() {
        return Math.toDegrees(wristMotor.getAngleRads());
    }

}
