
package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.extensions.TuneableArmFeedforward;
import frc.lib.tuneables.extensions.TuneableTrapezoidProfile;
import frc.robot.Robot;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOSim;
import frc.robot.subsystems.intake.io.IntakeIOSparkMax;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final IntakeIO io = Robot.isSimulation()
            ? new IntakeIOSim(fieldsTable)
            : new IntakeIOSparkMax(fieldsTable);

    private final TuneableTrapezoidProfile trapezoidProfile = new TuneableTrapezoidProfile(
            new TrapezoidProfile.Constraints(WRIST_MAX_VELOCITY_DEG_PER_SEC, WRIST_MAX_ACCELERATION_DEG_PER_SEC));

    private final TuneableArmFeedforward feedForwardWrist = new TuneableArmFeedforward(KS, KG, KV, KA);
    private final PIDController wristPidController = new PIDController(KP, KI, KD);

    SlewRateLimiter rollersSpeedLimiter = new SlewRateLimiter(ROLLERS_ACCELERATION_LIMIT_VOLTAGE_PER_SECOND);

    public Intake() {
        fieldsTable.update();
    }

    @Override
    public void periodic() {
    }

    public void setSpeedRollers(double speedPrecentageOutput) {
        speedPrecentageOutput = rollersSpeedLimiter.calculate(speedPrecentageOutput);
        io.setRollerSpeedPrecentOutput(MathUtil.clamp(
                speedPrecentageOutput,
                -ROLLERS_SPEED_LIMIT_PRECENTAGE,
                ROLLERS_SPEED_LIMIT_PRECENTAGE));
    }

    public void setWristVoltage(double voltage) {
        fieldsTable.recordOutput("Demand Voltage", voltage);
        voltage = MathUtil.clamp(voltage, -WRIST_VOLTAGE_LIMIT, WRIST_VOLTAGE_LIMIT);
        fieldsTable.recordOutput("Actual Voltage", voltage);
        io.setWristVoltage(voltage);
    }

    public double getAbsoluteAngleDegrees() {
        return io.wristAngleDegrees.getAsDouble();
    }

    public boolean getIsNoteInside() {
        return io.noteDetectorValue.getAsBoolean();
    }

    public double calculateFeedforward(double desiredWristAngleDegrees, double desiredWristVelocity, boolean usePID) {
        double voltages = feedForwardWrist.calculate(Math.toRadians(desiredWristAngleDegrees), desiredWristVelocity);

        if (usePID) {
            voltages += wristPidController.calculate(getAbsoluteAngleDegrees(), desiredWristAngleDegrees);
        }

        fieldsTable.recordOutput("feedforward result", voltages);
        return voltages;
    }

    public TrapezoidProfile.State calculateTrapezoidProfile(
            double time,
            TrapezoidProfile.State initialState,
            TrapezoidProfile.State goalState) {
        return trapezoidProfile.calculate(time, initialState, goalState);
    }
}