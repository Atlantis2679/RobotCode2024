// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.COLLECTING_POSITION_DEGREE;
import static frc.robot.subsystems.intake.IntakeConstants.COLLECTING_POSITION_INTAKE_SPEED;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class CollectingPosition extends ParallelCommandGroup {

  public CollectingPosition(Intake intake) {
  
    addCommands(
    new IntakeMoveToAngle(intake, COLLECTING_POSITION_DEGREE),
    new intakeButton(intake, COLLECTING_POSITION_INTAKE_SPEED)
    );
  }
}
