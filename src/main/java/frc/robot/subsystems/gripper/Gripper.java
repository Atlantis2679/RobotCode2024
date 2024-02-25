package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.gripper.GripperConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.subsystems.gripper.io.GripperIO;
import frc.robot.subsystems.gripper.io.GripperIOSparkMax;

public class Gripper extends SubsystemBase {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());

    private final GripperIO io = new GripperIOSparkMax(fieldsTable);

    SlewRateLimiter rollersSpeedLimiter = new SlewRateLimiter(GRIPPER_ACCELERATION_LIMIT_PRECENTAGE_PER_SECOND);

    public Gripper() {
        fieldsTable.update();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("gripper note inside", getIsNoteInside());
        fieldsTable.recordOutput("Is Note inside", getIsNoteInside());
        fieldsTable.recordOutput("current command", getCurrentCommand() != null ? getCurrentCommand().getName() : null);
    }

    public void setSpeed(double speedPrecentageOutput) {
        speedPrecentageOutput = rollersSpeedLimiter.calculate(speedPrecentageOutput);
        io.setGripperSpeedPrecentOutput(MathUtil.clamp(
                speedPrecentageOutput,
                -GRIPPER_SPEED_LIMIT_PRECENTAGE,
                GRIPPER_SPEED_LIMIT_PRECENTAGE));
    }

    public void stop() {
        io.setGripperSpeedPrecentOutput(0);
    }

    public boolean getIsNoteInside() {
        return io.noteDetectorValue.getAsBoolean();
    }
}
