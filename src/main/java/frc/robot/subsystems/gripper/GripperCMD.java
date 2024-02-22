// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class GripperCMD {
    private final Gripper gripper;

    public GripperCMD(Gripper gripper) {
        this.gripper = gripper;
    }

    public Command setGripperSpeed(double gripperSpeed) {
        return gripper.runOnce(() -> gripper.setGripperSpeed(gripperSpeed));
    }

    public Command manualController(DoubleSupplier speed){
        return gripper.run(()-> gripper.setGripperSpeed(speed.getAsDouble()));
    }

}