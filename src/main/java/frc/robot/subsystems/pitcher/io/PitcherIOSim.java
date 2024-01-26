package frc.robot.subsystems.pitcher.io;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.subsystems.pitcher.PitcherConstants.*;

public class PitcherIOSim extends PitcherIO {
    private SingleJointedArmSim motor = new SingleJointedArmSim(DCMotor.getNEO(1), GEAR_RATIO, 0.0001, 0.8, -Math.PI, Math.PI, true, 0);

    public PitcherIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected void periodicBeforeFields() {
        motor.update(0.02);
    }

    @Override
    protected double getAngleDegrees() {
        return Math.toDegrees(motor.getAngleRads());
    }

    @Override
    public void setVoltage(double demandVoltage) {
        motor.setInputVoltage(demandVoltage);
    }
}
