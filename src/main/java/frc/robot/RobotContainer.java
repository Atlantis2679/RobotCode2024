package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.robot.subsystems.pitcher.Pitcher;
import frc.robot.subsystems.pitcher.PitcherCommands;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utils.NaturalXboxController;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Pitcher pitcher = new Pitcher();
    private final NaturalXboxController driverController = new NaturalXboxController(RobotMap.Controllers.DRIVER_PORT);
    private final SwerveCommands swerveCommands = new SwerveCommands(swerve);
    private final PitcherCommands pitcherCommands = new PitcherCommands(pitcher);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        TuneableCommand driveCommand = swerveCommands.controller(
                driverController::getLeftY,
                driverController::getLeftX,
                new NaturalXboxController(1)::getLeftX,
                driverController.leftBumper().negate()::getAsBoolean);

        swerve.setDefaultCommand(driveCommand);
        TuneablesManager.add("Swerve/drive command", driveCommand.fullTuneable());

        driverController.a().onTrue(new InstantCommand(swerve::resetYaw));

        TuneablesManager.add("Swerve/modules control mode",
                swerveCommands.controlModules(
                        driverController::getLeftX,
                        driverController::getLeftY,
                        driverController::getRightX).fullTuneable());

        driverController.a().onTrue(pitcherCommands.adjustToAngle(90));
        driverController.y().onTrue(pitcherCommands.adjustToAngle(0));
        driverController.b().whileTrue(pitcherCommands.adjustToAngle(() -> driverController.getLeftY() * 90));
        pitcher.setDefaultCommand(pitcherCommands.controller(() -> driverController.getLeftY()));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
