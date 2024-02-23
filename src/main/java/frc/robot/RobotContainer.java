package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.robot.allcommands.AllCommands;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.loader.Loader;
import frc.robot.subsystems.pitcher.Pitcher;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.utils.NaturalXboxController;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Pitcher pitcher = new Pitcher();
    private final Wrist wrist = new Wrist();
    private final Gripper gripper = new Gripper();
    private final Flywheel flywheel = new Flywheel();
    private final Loader loader = new Loader();

    private final NaturalXboxController driverController = new NaturalXboxController(RobotMap.Controllers.DRIVER_PORT);
    private final NaturalXboxController operatorController = new NaturalXboxController(
            RobotMap.Controllers.OPERTATOR_PORT);

    private final SwerveCommands swerveCommands = new SwerveCommands(swerve);
    private final AllCommands allCommands = new AllCommands(swerve, flywheel, pitcher, loader, wrist, gripper);

    public RobotContainer() {
        new Trigger(DriverStation::isDisabled).onTrue(allCommands.stopAll());
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        TuneableCommand driveCommand = swerveCommands.controller(
                driverController::getLeftY,
                driverController::getLeftX,
                driverController::getRightX,
                driverController.leftBumper().negate()::getAsBoolean);

        swerve.setDefaultCommand(driveCommand);
        TuneablesManager.add("Swerve/drive command", driveCommand.fullTuneable());
        driverController.a().onTrue(new InstantCommand(swerve::resetYaw));
        driverController.y().onTrue(swerveCommands.xWheelLock());

        TuneablesManager.add("Swerve/modules control mode",
                swerveCommands.controlModules(
                        driverController::getLeftX,
                        driverController::getLeftY,
                        driverController::getRightY).fullTuneable());
    }

    private void configureOperatorBindings() {
        operatorController.leftBumper().whileTrue(allCommands.manualShooter(
                operatorController::getLeftY,
                operatorController::getRightY,
                operatorController::getLeftTriggerAxis,
                operatorController::getRightTriggerAxis));

        operatorController.rightBumper()
                .whileTrue(allCommands.manualIntake(operatorController::getLeftY, operatorController::getRightY));

        Command handoffCMD = allCommands.handoff();
        wrist.setDefaultCommand(Commands.either(
                Commands.runOnce(() -> handoffCMD.schedule()),
                allCommands.closeWrist(),
                gripper::getIsNoteInside));

        operatorController.a().whileTrue(allCommands.openIntake());

        pitcher.setDefaultCommand(Commands.either(
                allCommands.pitcherWithNoteIdle(),
                allCommands.pitcherReadyToHandOff(),
                loader::getIsNoteInside));

        operatorController.povUp().whileTrue(allCommands.readyToShootToSpeaker());
        operatorController.povDown().whileTrue(allCommands.readyToShootToAmp());
        operatorController.b().whileTrue(allCommands.shoot());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
