
package frc.robot.subsystems.wrist;

import static frc.robot.subsystems.wrist.WristConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.Tuneable;
import edu.wpi.first.util.sendable.Sendable;
import frc.lib.tuneables.TuneableBuilder;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.tuneables.extensions.TuneableArmFeedforward;
import frc.lib.tuneables.extensions.TuneableTrapezoidProfile;
import frc.robot.Robot;
import frc.robot.subsystems.wrist.WristConstants.IsAtAngle;
import frc.robot.subsystems.wrist.WristConstants.Sim;
import frc.robot.subsystems.wrist.io.WristIO;
import frc.robot.subsystems.wrist.io.WristIOSim;
import frc.robot.subsystems.wrist.io.WristIOSparkMax;
import frc.robot.utils.PrimitiveRotationalSensorHelper;

public class Wrist extends SubsystemBase implements Tuneable {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final WristIO io = Robot.isSimulation()
            ? new WristIOSim(fieldsTable)
            : new WristIOSparkMax(fieldsTable);

    private final PrimitiveRotationalSensorHelper wristAngleHelperDegrees;
    private final TuneableTrapezoidProfile trapezoidProfile = new TuneableTrapezoidProfile(
            new TrapezoidProfile.Constraints(WRIST_MAX_VELOCITY_DEG_PER_SEC, WRIST_MAX_ACCELERATION_DEG_PER_SEC));

    private final TuneableArmFeedforward feedForwardWrist = Robot.isSimulation()
            ? new TuneableArmFeedforward(KS, KG, KV, KA)
            : new TuneableArmFeedforward(Sim.KS, Sim.KG, Sim.KV, Sim.KA);
    private final PIDController wristPidController = new PIDController(KP, KI, KD);

    private final WristVisualizer realStateVisualizer = new WristVisualizer(
            fieldsTable,
            new Color8Bit("#00BEBE"),
            "Real Mechanism");

    private final WristVisualizer desiredStateVisualizer = new WristVisualizer(
            fieldsTable,
            new Color8Bit("#FFFF00"),
            "Desired Mechanism");

    public Wrist() {
        fieldsTable.update();
        wristAngleHelperDegrees = new PrimitiveRotationalSensorHelper(
                io.wristAngleDegrees.getAsDouble(),
                WIRST_ANGLE_OFFSET_DEGREES);

        wristAngleHelperDegrees.enableContinousWrap(WIRST_ANGLE_UPPER_BOUND_DEGREES, 360);
        TuneablesManager.add("Wrist", (Tuneable) this);
    }

    @Override
    public void periodic() {
        wristAngleHelperDegrees.update(io.wristAngleDegrees.getAsDouble());
        realStateVisualizer.update(getAbsoluteAngleDegrees());
        fieldsTable.recordOutput("Wrist Angle Degrees", getAbsoluteAngleDegrees());
    }

    public void setWristVoltage(double voltage) {
        fieldsTable.recordOutput("Demand Voltage", voltage);
        voltage = MathUtil.clamp(voltage, -WRIST_VOLTAGE_LIMIT, WRIST_VOLTAGE_LIMIT);
        fieldsTable.recordOutput("Actual Voltage", voltage);
        io.setWristVoltage(voltage);
    }

    public double getAbsoluteAngleDegrees() {
        return wristAngleHelperDegrees.getAngle();
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

    public boolean isAtAngle(double desiredAngle) {
        if (desiredAngle - getAbsoluteAngleDegrees() > IsAtAngle.MIN_WRIST_ANGLE_DEVAITION
                && desiredAngle - getAbsoluteAngleDegrees() < IsAtAngle.MAX_WRIST_ANGLE_DEVAITION)
            return true;
        return false;
    }

    public void stop() {
        setWristVoltage(0);
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        builder.addChild("Intake Subsystem", (Sendable) this);

        builder.addChild("Wrist PID", wristPidController);

        builder.addChild("Wrist feedforward", feedForwardWrist);

        builder.addChild("Trapezoid profile", trapezoidProfile);

        builder.addChild("wrist angle degrees", wristAngleHelperDegrees);
    }
}
