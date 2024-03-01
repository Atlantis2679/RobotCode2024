package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.robot.allcommands.AllCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.LedsCommands;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.utils.NaturalXboxController;

public class RobotContainer {
        private final Swerve swerve = new Swerve();
        private final Wrist wrist = new Wrist();
        private final Gripper gripper = new Gripper();
        private final Elevator elevator = new Elevator();
        private final Leds leds = new Leds();

        private final NaturalXboxController driverController = new NaturalXboxController(
                        RobotMap.Controllers.DRIVER_PORT);
        private final NaturalXboxController operatorController = new NaturalXboxController(
                        RobotMap.Controllers.OPERTATOR_PORT);

        private final LedsCommands ledsCommands = new LedsCommands(leds);
        private final SwerveCommands swerveCommands = new SwerveCommands(swerve);
        private final AllCommands allCommands = new AllCommands(swerve, wrist, gripper, elevator);

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
                        CvSink cvSink = CameraServer.getVideo();
                        CvSource outputStream = CameraServer.putVideo("blur", 640, 480);
                }

                // ---- first auto command chooser ----

                firstAutoCommandChooser.addDefaultOption("None", () -> new InstantCommand());

                firstAutoCommandChooser.addOption("Eject Note",
                                () -> allCommands.getReadyToScoreAMP().alongWith(allCommands.scoreAMP()));

                // ---- second auto command chooser ----

                secondAutoCommandChooser.addDefaultOption("None", () -> new InstantCommand());

                secondAutoCommandChooser.addOption("GetToAmp", () -> new PathPlannerAuto("GetToAmp"));

                secondAutoCommandChooser.addOption("GetOutFromSource", () -> new PathPlannerAuto("GetOutFromSource"));

                secondAutoCommandChooser.addOption("MiddleSpeakerAndGetOut",
                                () -> new PathPlannerAuto("MiddleSpeakerAndGetOut"));

                secondAutoCommandChooser.addOption("MiddleSpeaker", () -> new PathPlannerAuto("MiddleSpeaker"));

                secondAutoCommandChooser.addOption("SpeakerCloseToAmpAndGetOut",
                                () -> new PathPlannerAuto("SpeakerCloseToAmpAndGetOut"));

                secondAutoCommandChooser.addOption("SpeakerCloseToAmp", () -> new PathPlannerAuto("SpeakerCloseToAmp"));

                secondAutoCommandChooser.addOption("SpeakerFarFromAmpAndGetOut",
                                () -> new PathPlannerAuto("SpeakerFarFromAmpAndGetOut"));
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

                driverController.leftTrigger()
                                .onTrue(allCommands.manualElevator(driverController::getLeftTriggerAxis, () -> true));
                driverController.rightTrigger()
                                .onTrue(allCommands.manualElevator(driverController::getRightTriggerAxis, () -> false));

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
                operatorController.rightBumper()
                                .whileTrue(allCommands.manualIntake(null, null, null));

                wrist.setDefaultCommand(allCommands.closeWrist().withName("wrist default"));

                operatorController.a().whileTrue(allCommands.openIntake());

                operatorController.leftTrigger().whileTrue(allCommands.getReadyToScoreAMP());
                operatorController.b().whileTrue(allCommands.scoreAMP());

                operatorController.povUp().onTrue(allCommands.changeCounter(() -> true));
                operatorController.povDown().onTrue(allCommands.changeCounter(() -> false));

                TuneableCommand tuneableReadyToShootCMD = allCommands.readyToShootTuneable();
                operatorController.povLeft().and(TuneablesManager::isEnabled).whileTrue(tuneableReadyToShootCMD);
                TuneablesManager.add("tuneable ready to shoot", (Tuneable) tuneableReadyToShootCMD);
        }

        public void configureNamedCommands() {
        }

        public void configureLeds() {
                operatorController.a()
                                .whileTrue(gripper.getIsNoteInside() ? ledsCommands.setGreen() : ledsCommands.setRed());
                operatorController.a().whileFalse(ledsCommands.set00BEBE());
        }

        public Command getAutonomousCommand() {
                return firstAutoCommandChooser.get().get()
                                .andThen(secondAutoCommandChooser.get().get());
        }
}