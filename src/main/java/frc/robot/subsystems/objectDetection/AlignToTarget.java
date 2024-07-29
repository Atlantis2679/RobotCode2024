// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectDetection;

import edu.wpi.first.wpilibj2.command.Command;

public class AlignToTarget extends Command {
  /** Creates a new AlignToTarget. */
  private final ObjectDetection objectDetection;

  public AlignToTarget(ObjectDetection objectDetection) {
    this.objectDetection = objectDetection;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(objectDetection.hasTarget());
    double yaw = (objectDetection.getBestResult()).getYaw();

    //for next time
    //https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html#what-is-a-photon-tracked-target
    //https://docs.photonvision.org/en/latest/docs/integration/simpleStrategies.html
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
