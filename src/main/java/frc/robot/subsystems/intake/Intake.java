
package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.Tuneable;
import edu.wpi.first.util.sendable.Sendable;
import frc.lib.tuneables.TuneableBuilder;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.tuneables.extensions.TuneableArmFeedforward;
import frc.lib.tuneables.extensions.TuneableTrapezoidProfile;
import frc.lib.valueholders.DoubleHolder;
import frc.robot.Robot;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOSim;
import frc.robot.subsystems.intake.io.IntakeIOSparkMax;
import frc.robot.utils.RotationalSensorHelper;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class Intake extends SubsystemBase implements Tuneable {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final IntakeIO io = Robot.isSimulation()
            ? new IntakeIOSim(fieldsTable)
            : new IntakeIOSparkMax(fieldsTable);

    private final RotationalSensorHelper wristAngleHelper;
    private final TuneableTrapezoidProfile trapezoidProfile = new TuneableTrapezoidProfile(
            new TrapezoidProfile.Constraints(WRIST_MAX_VELOCITY_DEG_PER_SEC, WRIST_MAX_ACCELERATION_DEG_PER_SEC));

    private final TuneableArmFeedforward feedForwardWrist = new TuneableArmFeedforward(KS, KG, KV, KA);
    private final PIDController wristPidController = new PIDController(KP, KI, KD);

    private final IntakeVisualizer realStateVisualizer = new IntakeVisualizer(
            fieldsTable,
            new Color8Bit("#00BEBE"),
            "Real Mechanism");

    private final IntakeVisualizer desiredStateVisualizer = new IntakeVisualizer(
            fieldsTable,
            new Color8Bit("#FFFF00"),
            "Desired Mechanism");

    SlewRateLimiter rollersSpeedLimiter = new SlewRateLimiter(ROLLERS_ACCELERATION_LIMIT_VOLTAGE_PER_SECOND);

    public Intake() {
        fieldsTable.update();
        wristAngleHelper = new RotationalSensorHelper(
                Rotation2d.fromDegrees(io.wristAngleDegrees.getAsDouble()),
                Rotation2d.fromDegrees(WIRST_ANGLE_OFFSET_DEGREES));

        wristAngleHelper.enableContinousWrap(Rotation2d.fromDegrees(WIRST_ANGLE_UPPER_BOUND_DEGREES));
        TuneablesManager.add("Intake", (Tuneable) this);
    }

    @Override
    public void periodic() {
        wristAngleHelper.update(Rotation2d.fromDegrees(io.wristAngleDegrees.getAsDouble()));
        realStateVisualizer.update(getAbsoluteAngleDegrees());
        fieldsTable.recordOutput("Wrist Angle Degrees", getAbsoluteAngleDegrees());
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
        return wristAngleHelper.getAngle().getDegrees();
    }

    public boolean getIsNoteInside() {
        return io.noteDetectorValue.getAsBoolean();
    }

    public double calculateFeedforward(double desiredWristAngleDegrees, double desiredWristVelocity, boolean usePID) {
        fieldsTable.recordOutput("desiredWristAngleDegrees", desiredWristAngleDegrees);
        ;
        double voltages = feedForwardWrist.calculate(Math.toRadians(desiredWristAngleDegrees), desiredWristVelocity);

        if (usePID) {
            voltages += wristPidController.calculate(getAbsoluteAngleDegrees(), desiredWristAngleDegrees);
        }

        fieldsTable.recordOutput("feedforward result", voltages);
        return voltages;
    }

    public void resetPID() {
        wristPidController.reset();
    }

    public TrapezoidProfile.State calculateTrapezoidProfile(
            double time,
            TrapezoidProfile.State initialState,
            TrapezoidProfile.State goalState) {

        TrapezoidProfile.State state = trapezoidProfile.calculate(time, initialState, goalState);
        fieldsTable.recordOutput("desiredState", state.position);
        fieldsTable.recordOutput("desired velocity", state.velocity);
        desiredStateVisualizer.update(state.position);

        return state;
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        builder.addChild("Intake Subsystem", (Sendable) this);

        builder.addChild("Wrist PID", wristPidController);

        builder.addChild("Wrist feedforward", feedForwardWrist);

        builder.addChild("Trapezoid profile", trapezoidProfile);

        builder.addChild("wrist angle degrees", (Tuneable) (wristAngleBuilder) -> {
            DoubleHolder angleToResetDegrees = new DoubleHolder(0);
            wristAngleBuilder.addDoubleProperty("raw angle measurment",
                    () -> wristAngleHelper.getMeasuredAngle().getDegrees(), null);

            wristAngleBuilder.addDoubleProperty("calculated angle", this::getAbsoluteAngleDegrees, null);
            wristAngleBuilder.addDoubleProperty("offset", () -> wristAngleHelper.getOffset().getDegrees(), null);

            wristAngleBuilder.addDoubleProperty("angle to reset", angleToResetDegrees::get,
                    angleToResetDegrees::set);

            wristAngleBuilder.addChild("reset!", new InstantCommand(() -> {
                wristAngleHelper.resetAngle(Rotation2d.fromDegrees(angleToResetDegrees.get()));
            }));
        });
    }

}
