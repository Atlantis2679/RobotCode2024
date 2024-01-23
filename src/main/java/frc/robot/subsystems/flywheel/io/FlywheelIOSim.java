package frc.robot.subsystems.flywheel.io;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.subsystems.flywheel.FlywheelConstants.*;

public class FlywheelIOSim extends FlywheelIO {
    private FlywheelSim upperRollerSim = new FlywheelSim(DCMotor.getNEO(1), UPPER_ROLLER_GEAR_RATIO, 0.003);
    private FlywheelSim lowerRollerSim = new FlywheelSim(DCMotor.getNEO(1), LOWER_ROLLER_GEAR_RATIO, 0.003);

    public FlywheelIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected void periodicBeforeFields() {
        lowerRollerSim.update(0.02);
        upperRollerSim.update(0.02);
    }

    @Override
    protected double getUpperRollerSpeedRPS() {
        return upperRollerSim.getAngularVelocityRPM() / 60;
    }

    @Override
    protected double getLowerRollerSpeedRPS() {
        return lowerRollerSim.getAngularVelocityRPM() / 60;
    }

    @Override
    public void setUpperRollerVoltage(double voltageDemand) {
        upperRollerSim.setInputVoltage(voltageDemand);
    }

    @Override
    public void setLowerRollerVoltage(double voltageDemand) {
        lowerRollerSim.setInputVoltage(voltageDemand);
    }
}
