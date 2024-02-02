// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;
import static frc.robot.subsystems.intake.IntakeConstants.*;


import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class IntakeCommends {

    private final Intake intake;

    public IntakeCommends(Intake intake){
    this.intake = intake;
    }


    public Command collectingPosition(){
        return new IntakeMoveToAngle(intake, COLLECTING_POSITION_DEGREE).alongWith(
               new IntakeButton(intake, COLLECTING_POSITION_ROLLERS_SPEED));
    }

    public Command passToShooter(){
        if (intake.getBeamBreakValue()) {
             return new IntakeMoveToAngle(intake, PASSING_NOTE_TO_SHOOTER_WRIST_DEGREE).andThen(
                    new IntakeButton(intake, PASSING_NOTE_TO_SHOOTER_ROLLERS_SPEED));
    }

    return new IntakeMoveToAngle(intake, PASSING_NOTE_TO_SHOOTER_WRIST_DEGREE);
    }
    
    public Command ampPosition(){
        return new IntakeMoveToAngle(intake, AIM_TO_AMP_WRIST_DEGREE);
        //incase we need it
    }
    
}
