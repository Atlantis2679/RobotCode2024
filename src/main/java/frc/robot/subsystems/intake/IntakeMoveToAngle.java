
package frc.robot.subsystems.intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakeMoveToAngle extends Command {
  private final Intake intake;
  private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      MAX_VELOCITY_JOINT,
      WRIST_MAX_ACCELERATION));
  private final double desiredJointAngle;
  private final Timer timer = new Timer();

  public IntakeMoveToAngle(Intake intake, double desiredJointAngleDegree) {
    this.intake = intake;
    this.desiredJointAngle = desiredJointAngleDegree;
    addRequirements(intake);

  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    intake.calculateFeedforward(desiredJointAngle, MAX_VELOCITY_JOINT, true);

    trapezoidProfile.calculate(timer.get(),
        new TrapezoidProfile.State(intake.getAbsoluteAngle(), intake.getArmSpeed()),
        new TrapezoidProfile.State(desiredJointAngle, 0));
  }

  @Override
  public void end(boolean interrupted) {
    intake.setAngleIntake(STOP_INTAKE_ANGLE_MOTOR);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
