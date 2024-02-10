package frc.robot.subsystems.flywheel;

import static frc.robot.subsystems.flywheel.FlywheelConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.tuneables.extensions.TuneableSimpleMotorFeedforward;
import frc.robot.Robot;
import frc.robot.subsystems.flywheel.io.FlywheelIO;
import frc.robot.subsystems.flywheel.io.FlywheelIOSim;
import frc.robot.subsystems.flywheel.io.FlywheelIOSparkMax;

import static frc.robot.subsystems.flywheel.FlywheelConstants.RotateFlywheel.*;

public class Flywheel extends SubsystemBase implements Tuneable {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final FlywheelIO io = Robot.isSimulation()
            ? new FlywheelIOSim(fieldsTable)
            : new FlywheelIOSparkMax(fieldsTable);

    private final SlewRateLimiter upperRollerSpeedAccelerationLimiter = new SlewRateLimiter(
            MAX_ACCELERATION_VOLTS_PER_SECOND);
    private final SlewRateLimiter lowerRollerSpeedAccelerationLimiter = new SlewRateLimiter(
            MAX_ACCELERATION_VOLTS_PER_SECOND);

    private final PIDController upperRollerSpeedPID = new PIDController(KP, KI, KD);
    private final PIDController lowerRollerSpeedPID = new PIDController(KP, KI, KD);

    private final TuneableSimpleMotorFeedforward feedforward = new TuneableSimpleMotorFeedforward(KS, KV, KA);

    public Flywheel() {
        fieldsTable.update();

        TuneablesManager.add("Flywheel", (Tuneable) this);
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

    public void resetPIDs() {
        lowerRollerSpeedPID.reset();
        upperRollerSpeedPID.reset();
    }

    public double calculateUpperRollerVoltageForSpeed(double velocityRPS) {
        double result = feedforward.calculate(velocityRPS)
                + upperRollerSpeedPID.calculate(getUpperRollerSpeedRPS(), velocityRPS);
        return result;
    }

    public double calculateLowerRollerVoltageForSpeed(double velocityRPS) {
        double result = feedforward.calculate(velocityRPS)
                + lowerRollerSpeedPID.calculate(getLowerRollerSpeedRPS(), velocityRPS);
        return result;
    }

    public double getUpperRollerSpeedRPS() {
        return io.upperRollerSpeedRPS.getAsDouble();
    }

    public double getLowerRollerSpeedRPS() {
        return io.lowerRollerSpeedRPS.getAsDouble();
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        builder.addChild("feedforward", feedforward);
        builder.addChild("Upper PID", upperRollerSpeedPID);
        builder.addChild("Lower PID", lowerRollerSpeedPID);
    }
}
