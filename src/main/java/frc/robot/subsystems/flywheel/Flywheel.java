package frc.robot.subsystems.flywheel;

import static frc.robot.subsystems.flywheel.FlywheelConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.Robot;
import frc.robot.subsystems.flywheel.io.FlywheelIO;
import frc.robot.subsystems.flywheel.io.FlywheelIOSim;
import frc.robot.subsystems.flywheel.io.FlywheelIOSparkMax;

public class Flywheel extends SubsystemBase {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final FlywheelIO io = Robot.isSimulation()
            ? new FlywheelIOSim(fieldsTable)
            : new FlywheelIOSparkMax(fieldsTable);

    private final SlewRateLimiter upperRollerSpeedAccelerationLimiter = new SlewRateLimiter(
            MAX_ACCELERATION_VOLTS_PER_SECOND);
    private final SlewRateLimiter lowerRollerSpeedAccelerationLimiter = new SlewRateLimiter(
            MAX_ACCELERATION_VOLTS_PER_SECOND);

    public Flywheel() {
        fieldsTable.update();
    }

    @Override
    public void periodic() {
    }

    public void setSpeed(double upperRollerDemandVoltage, double lowerRollerDemandVoltage) {
        upperRollerDemandVoltage = upperRollerSpeedAccelerationLimiter.calculate(upperRollerDemandVoltage);
        lowerRollerDemandVoltage = lowerRollerSpeedAccelerationLimiter.calculate(lowerRollerDemandVoltage);
        io.setUpperRollerVoltage(MathUtil.clamp(upperRollerDemandVoltage, -MAX_VOLTAGE, MAX_VOLTAGE));
        io.setLowerRollerVoltage(MathUtil.clamp(lowerRollerDemandVoltage, -MAX_VOLTAGE, MAX_VOLTAGE));
    }

    public double getUpperRollerSpeedRPS() {
        return io.upperRollerSpeedRPS.getAsDouble();
    }

    public double getLowerRollerSpeedRPS() {
        return io.lowerRollerSpeedRPS.getAsDouble();
    }

    public boolean atSpeed(double upperRollerRPS, double lowerRollerRPS) {
        return Math.abs(getUpperRollerSpeedRPS() - upperRollerRPS) < SPEED_TOLERANCE_RPS
                && Math.abs(getLowerRollerSpeedRPS() - lowerRollerRPS) < SPEED_TOLERANCE_RPS;
    }
}
