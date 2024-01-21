
package frc.robot.subsystems.intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeMoveToAngle extends Command {
  private final Intake intake;
 // private final TrapezoidProfile trapezoidProfile;


 

  public IntakeMoveToAngle(Intake intake, double angle){
    this.intake = intake;

   // trapezoidProfile.calculate(0, null, null);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.calculateFeedforward(IntakeConstants.ARM_ANGLE, IntakeConstants.INTAKE_ARM_VELOCITY, true);

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
