package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.tuneables.Tuneable;
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

        private final NaturalXboxController driverController = new NaturalXboxController(
                        RobotMap.Controllers.DRIVER_PORT);
        private final NaturalXboxController operatorController = new NaturalXboxController(
                        RobotMap.Controllers.OPERTATOR_PORT);

        private final SwerveCommands swerveCommands = new SwerveCommands(swerve);
        private final AllCommands allCommands = new AllCommands(swerve, flywheel, pitcher, loader, wrist, gripper);

        private final LoggedDashboardChooser<Supplier<Command>> firstAutoCommandChooser = new LoggedDashboardChooser<>(
                        "First Auto Command");

        private final LoggedDashboardChooser<Supplier<Command>> secondAutoCommandChooser = new LoggedDashboardChooser<>(
                        "Second Auto Command");

        public RobotContainer() {
                new Trigger(DriverStation::isDisabled).onTrue(allCommands.stopAll());
                configureDriverBindings();
                configureOperatorBindings();
                configureNamedCommands();

                if (Robot.isReal()) {
                        CameraServer.startAutomaticCapture();
                }

                firstAutoCommandChooser.addDefaultOption("None", () -> new InstantCommand());

                firstAutoCommandChooser.addOption("Shoot Note to Speaker", () -> allCommands.readyToShootToSpeaker()
                                .raceWith(allCommands.shoot().until(() -> !loader.getIsNoteInside())));

                secondAutoCommandChooser.addDefaultOption("None", () -> new InstantCommand());

                secondAutoCommandChooser.addOption("Get Out of Staring Line",
                                () -> new PathPlannerAuto("getOutOfStartingLine"));

                secondAutoCommandChooser.addOption("Speaker Close To Amp",
                                () -> new PathPlannerAuto("SpeakerCloseToAmp"));
                secondAutoCommandChooser.addOption("Speaker Close To Amp And Out",
                                () -> new PathPlannerAuto("SpeakerCloseToAmpAndGetOut"));

                secondAutoCommandChooser.addOption("Speaker Far From Amp",
                                () -> new PathPlannerAuto("SpeakerFarFromAmp"));
                secondAutoCommandChooser.addOption("Speaker Far From Amp And Out",
                                () -> new PathPlannerAuto("SpeakerFarFromAmpAndGetOut"));

                secondAutoCommandChooser.addOption("Middle Speaker",
                                () -> new PathPlannerAuto("MiddleSpeaker"));
                secondAutoCommandChooser.addOption("Middle Speaker And Out",
                                () -> new PathPlannerAuto("MiddleSpeakerAndGetOut"));
        }

        private void configureDriverBindings() {
                TuneableCommand driveCommand = swerveCommands.controller(
                                () -> driverController.getLeftY(),
                                () -> driverController.getLeftX(),
                                () -> driverController.getRightX(),
                                driverController.leftBumper().negate()::getAsBoolean);

                swerve.setDefaultCommand(driveCommand);
                TuneablesManager.add("Swerve/drive command", driveCommand.fullTuneable());
                driverController.a().onTrue(new InstantCommand(swerve::resetYaw));
                driverController.x().onTrue(swerveCommands.xWheelLock());

                TuneablesManager.add("Swerve/modules control mode",
                                swerveCommands.controlModules(
                                                driverController::getLeftX,
                                                driverController::getLeftY,
                                                driverController::getRightY).fullTuneable());

                // driverController.y().onTrue(Commands.runOnce(() -> {
                // swerve.resetPose(new Pose2d(new Translation2d(2, 7.22),
                // Rotation2d.fromDegrees(90)));
                // }));

                // driverController.x().whileTrue(swerveCommands.driveToPose(new Pose2d(2, 8.3,
                // Rotation2d.fromDegrees(90))));
        }

        private void configureOperatorBindings() {
                operatorController.leftBumper().whileTrue(allCommands.manualShooter(
                                operatorController::getLeftY,
                                operatorController::getRightY,
                                operatorController::getLeftTriggerAxis,
                                operatorController::getRightTriggerAxis));

                operatorController.rightBumper()
                                .whileTrue(allCommands.manualIntake(operatorController::getLeftY,
                                                operatorController::getRightY));

                Command autoHandoffCMD = allCommands.handoff().until(loader::getIsNoteInside).withName("auto handoff");
                wrist.setDefaultCommand(Commands.either(
                                Commands.runOnce(() -> autoHandoffCMD.schedule()),
                                allCommands.closeWrist(),
                                gripper::getIsNoteInside).withName("wrist default"));

                operatorController.y().whileTrue(allCommands.handoff());

                operatorController.a().whileTrue(allCommands.openIntake());

                pitcher.setDefaultCommand(Commands.either(
                                allCommands.pitcherWithNoteIdle(),
                                allCommands.pitcherReadyToHandOff(),
                                loader::getIsNoteInside).withName("pitcher default"));

                operatorController.povUp().whileTrue(allCommands.readyToShootToSpeaker());
                operatorController.povDown().whileTrue(allCommands.readyToShootToAmp());
                operatorController.b().whileTrue(allCommands.shoot());

                operatorController.x().onTrue(allCommands.stopAll());
                TuneableCommand tuneableReadyToShootCMD = allCommands.readyToShootTuneable();
                operatorController.povLeft().and(TuneablesManager::isEnabled).whileTrue(tuneableReadyToShootCMD);
                TuneablesManager.add("tuneable ready to shoot", (Tuneable) tuneableReadyToShootCMD);
        }

        public void configureNamedCommands() {
                NamedCommands.registerCommand("readyToShootToSpeaker", allCommands.readyToShootToSpeaker());
                NamedCommands.registerCommand("shootUntilStopSeeNote",
                                allCommands.shoot().until(() -> !loader.getIsNoteInside()));
                NamedCommands.registerCommand("stopAll", allCommands.stopAll());
                NamedCommands.registerCommand("openIntake", allCommands.openIntake());
        }

        public Command getAutonomousCommand() {
                return firstAutoCommandChooser.get().get()
                                .andThen(secondAutoCommandChooser.get().get());
        }
}
