// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class GripperCommands {
    private final Gripper gripper;

    public GripperCommands(Gripper gripper) {
        this.gripper = gripper;
    }

    public Command spin(double upperGripperSpeed, double lowerGripperSpeed) {
        return gripper.startEnd(() -> gripper.setSpeed(upperGripperSpeed, lowerGripperSpeed), gripper::stop);
    }

    public Command manualController(DoubleSupplier upperGripperSpeed, DoubleSupplier lowerGripperSpeed) {
        return gripper.run(() -> gripper.setSpeed(upperGripperSpeed.getAsDouble(), lowerGripperSpeed.getAsDouble()))
                .finallyDo(gripper::stop);
    }
}