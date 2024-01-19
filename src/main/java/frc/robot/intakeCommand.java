
package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class intakeCommand extends Command {
  private final Intake intake;
  private double startingPositionAngle;
  private final PIDController pidControllerIntake = new PIDController(
      IntakeConstants.KP,
      IntakeConstants.KI,
      IntakeConstants.KD);

  public intakeCommand(Intake intake) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    pidControllerIntake.setSetpoint(IntakeConstants.INTAKE_POSITION_ANGLE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setAngleIntake(pidControllerIntake.calculate(intake.getAbsoluteAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setAngleIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
