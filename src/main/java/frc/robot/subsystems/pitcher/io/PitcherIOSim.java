package frc.robot.subsystems.pitcher.io;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.subsystems.pitcher.PitcherConstants.*;

public class PitcherIOSim extends PitcherIO {
    private FlywheelSim motor = new FlywheelSim(DCMotor.getNEO(1), GEAR_RATIO, 0.003);
    private double angleDegrees = 0;

    public PitcherIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected void periodicBeforeFields() {
        motor.update(0.02);
        angleDegrees += (motor.getAngularVelocityRPM() / 60) * 0.02;
    }

    @Override
    protected double getAngleDegrees() {
        return angleDegrees;
    }

    @Override
    public void setVoltage(double demandVoltage) {
        motor.setInputVoltage(demandVoltage);
    }
}
