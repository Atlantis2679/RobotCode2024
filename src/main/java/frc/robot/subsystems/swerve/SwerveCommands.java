package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.lib.valueholders.BooleanHolder;
import frc.robot.subsystems.swerve.commands.SwerveController;

public class SwerveCommands {
    private final Swerve swerve;

    public SwerveCommands(Swerve swerve) {
        this.swerve = swerve;
    }

    public TuneableCommand controller(DoubleSupplier forwardSupplier, DoubleSupplier sidewaysSupplier,
            DoubleSupplier rotationSupplier, BooleanSupplier isFieldRelativeSupplier) {
        return new SwerveController(swerve, forwardSupplier, sidewaysSupplier, rotationSupplier,
                isFieldRelativeSupplier);
    }

    public TuneableCommand controlModules(DoubleSupplier steerXSupplier, DoubleSupplier steerYSupplier,
            DoubleSupplier speedSupplier) {
        return TuneableCommand.wrap(tuneableBuilder -> {
            BooleanHolder optimizeState = tuneableBuilder.addBoolean("optimize state", true);
            return new RunCommand(() -> {
                double steerY = steerYSupplier.getAsDouble();
                double steerX = steerXSupplier.getAsDouble();
                double speed = speedSupplier.getAsDouble();
                Logger.recordOutput("angle", steerX != 0 || steerY != 0
                                            ? Math.atan2(steerY, steerX) - Math.toRadians(90)
                                            : 0);
                SwerveModuleState[] moduleStates = new SwerveModuleState[4];
                for (int i = 0; i < moduleStates.length; i++) {
                    moduleStates[i] = new SwerveModuleState(
                            speed * SwerveContants.MAX_SPEED_MPS,
                            new Rotation2d(
                                    steerX != 0 || steerY != 0
                                            ? Math.atan2(steerY, steerX) - Math.toRadians(90)
                                            : 0));
                }
                swerve.setModulesState(moduleStates, false, optimizeState.get(), false);
            }, swerve);
        });
    }

    public Command xWheelLock() {
        return swerve.runOnce(() -> {
            SwerveModuleState[] moduleStates = new SwerveModuleState[4];
            moduleStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
            moduleStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
            moduleStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(135));
            moduleStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(-135));

            swerve.setModulesState(moduleStates, false, false, false);
        });
    }
}
