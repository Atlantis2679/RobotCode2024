package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
        public final AllCommands allCommands = new AllCommands(swerve, wrist, gripper, elevator);

        private final LoggedDashboardChooser<Supplier<Command>> firstAutoCommandChooser = new LoggedDashboardChooser<>(
                        "First Auto Command");

        private final LoggedDashboardChooser<Supplier<Command>> secondAutoCommandChooser = new LoggedDashboardChooser<>(
                        "Second Auto Command");

        private final LoggedDashboardChooser<Supplier<Command>> thirdAutoCommandChooser = new LoggedDashboardChooser<>(
                        "Third Auto Command");

        // private final LoggedDashboardChooser<Supplier<Command>> fourthAutoCommandChooser = new LoggedDashboardChooser<>(
        //                 "Fourth Auto Command");

        public RobotContainer() {
                new Trigger(DriverStation::isDisabled).onTrue(allCommands.stopAll());
                configureDriverBindings();
                configureOperatorBindings();
                configureNamedCommands();
                configureLeds();

                if (Robot.isReal()) {
                        // CameraServer.startAutomaticCapture();
                        // CvSink cvSink = CameraServer.getVideo();
                        // CvSource outputStream = CameraServer.putVideo("blur", 640, 480);
                        // UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
                        // MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
                        // mjpegServer1.setSource(usbCamera);
                        // CvSink cvSink = new CvSink("opencv_USB Camera 0");
                        // cvSink.setSource(usbCamera);
                        // CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480,
                        // 30);
                        // MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
                        // mjpegServer2.setSource(outputStream);
                }

                firstAutoCommandChooser.addDefaultOption("None", () -> new InstantCommand());

                firstAutoCommandChooser.addOption("Eject Note",
                                () -> allCommands.eject());

                firstAutoCommandChooser.addOption("Wait 7 Seconds", () -> Commands.waitSeconds(7));

                firstAutoCommandChooser.addOption("Wait 5 Seconds", () -> Commands.waitSeconds(5));

                firstAutoCommandChooser.addOption("Wait 3 Seconds", () -> Commands.waitSeconds(3));

                // ---- second auto command chooser ----

                secondAutoCommandChooser.addDefaultOption("None", () -> new InstantCommand());

                secondAutoCommandChooser.addOption("GetToAmpClose", () -> new PathPlannerAuto("GetToAmpClose"));

                secondAutoCommandChooser.addOption("GetToAmpFromCloseWall",
                                () -> new PathPlannerAuto("GetToAmpFromCloseWall"));

                secondAutoCommandChooser.addOption("GetOutFromSource", () -> new PathPlannerAuto("GetOutFromSource"));

                secondAutoCommandChooser.addOption("MiddleSpeakerAndGetOut",
                                () -> new PathPlannerAuto("MiddleSpeakerAndGetOut"));

                secondAutoCommandChooser.addOption("MiddleSpeaker", () -> new PathPlannerAuto("MiddleSpeaker"));

                secondAutoCommandChooser.addOption("SpeakerCloseToAmpAndGetOut",
                                () -> new PathPlannerAuto("SpeakerCloseToAmpAndGetOut"));

                secondAutoCommandChooser.addOption("SpeakerCloseToAmp", () -> new PathPlannerAuto("SpeakerCloseToAmp"));

                secondAutoCommandChooser.addOption("SpeakerFarFromAmpAndGetOut",
                                () -> new PathPlannerAuto("SpeakerFarFromAmpAndGetOut"));

                // ---- third auto command chooser ----

                thirdAutoCommandChooser.addDefaultOption("None", () -> new InstantCommand());

                thirdAutoCommandChooser.addOption("Score Amp", () -> Commands.waitSeconds(1)
                                .andThen(swerveCommands.xWheelLock().andThen(Commands
                                                .race(allCommands.getReadyToScoreAMP(), Commands.waitSeconds(3))
                                                .andThen(Commands.waitSeconds(1)
                                                                .andThen(allCommands.scoreAMPwithNoWaiting())))));

                // // ---- fourth auto command chooser ----

                // fourthAutoCommandChooser.addDefaultOption("None", () -> new InstantCommand());

                // fourthAutoCommandChooser.addOption("Get out of line after AMP", () -> new PathPlannerAuto("GetOutOfStartingLineAfterAMP"));
        }

        private void configureDriverBindings() {
                TuneableCommand driveCommand = swerveCommands.controller(
                                () -> driverController.getLeftY(),
                                () -> driverController.getLeftX(),
                                () -> driverController.getRightX(),
                                driverController.leftBumper().negate()::getAsBoolean,
                                driverController.rightBumper()::getAsBoolean);

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
                                .whileTrue(allCommands.manualIntake(operatorController::getRightY,
                                                () -> operatorController.getLeftTriggerAxis(),
                                                operatorController::getRightTriggerAxis));

                wrist.setDefaultCommand(allCommands.closeWrist().withName("wrist default"));

                operatorController.a().whileTrue(allCommands.openIntake());
                operatorController.y().whileTrue(allCommands.deliver());
                operatorController.povUp().whileTrue(allCommands.eject());
                operatorController.povDown().whileTrue(allCommands.eat());
                operatorController.leftBumper().whileTrue(allCommands.getReadyToScoreAMP());
                operatorController.b().whileTrue(allCommands.scoreAMP());
                operatorController.x().whileTrue(allCommands.makeSureNoteStaysInside());

                // TuneableCommand tuneableReadyToShootCMD = allCommands.scoreAMPTuenble();
                // operatorController.povLeft().and(TuneablesManager::isEnabled).whileTrue(tuneableReadyToShootCMD);
                // TuneablesManager.add("tuneable ready to shoot", (Tuneable)
                // tuneableReadyToShootCMD);
        }

        public void configureNamedCommands() {
        }

        public void configureLeds() {
                operatorController.a()
                                .onTrue(ledsCommands.setRed().until(() -> gripper.getIsNoteInside())
                                                .andThen(Commands.race(Commands.waitSeconds(5),
                                                                ledsCommands.setGreen())
                                                                .andThen(ledsCommands.set00BEBE())));

                operatorController.a().onFalse(ledsCommands.set00BEBE());
        }

        public Command getAutonomousCommand() {
                return firstAutoCommandChooser.get().get()
                                .andThen(secondAutoCommandChooser.get().get()
                                                .andThen(thirdAutoCommandChooser.get().get()));
        }
}