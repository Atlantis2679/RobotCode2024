package frc.robot.subsystems.swerve.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.TuneableBuilder;
import frc.lib.tuneables.TuneablesTable;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.lib.valueholders.DoubleHolder;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveContants;

import static frc.robot.subsystems.swerve.SwerveContants.*;

public class SwerveController extends TuneableCommand {
    private final Swerve swerve;
    private TuneablesTable tuneablesTable = new TuneablesTable(SendableType.LIST);

    private DoubleSupplier sidewaysSupplier;
    private DoubleSupplier forwardSupplier;
    private DoubleSupplier rotationsSupplier;
    private BooleanSupplier isFieldRelative;
    private BooleanSupplier isSensetiveMode;

    private DoubleHolder maxSpeedAngular = tuneablesTable.addNumber("Max Angular Velocity", MAX_ANGULAR_VELOCITY);
    private SendableChooser<Double> velocityMultiplierChooser = new SendableChooser<>();

    public SwerveController(Swerve swerve, DoubleSupplier forwardSupplier, DoubleSupplier sidewaysSupplier,
            DoubleSupplier rotationsSupplier, BooleanSupplier isFieldRelative, BooleanSupplier isSensetiveMode) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.sidewaysSupplier = sidewaysSupplier;
        this.forwardSupplier = forwardSupplier;
        this.rotationsSupplier = rotationsSupplier;
        this.isFieldRelative = isFieldRelative;
        this.isSensetiveMode = isSensetiveMode;

        velocityMultiplierChooser.addOption("REGULAR (100%)", 1.0);
        velocityMultiplierChooser.addOption("CHILD (70%)", 0.7);
        velocityMultiplierChooser.addOption("BABY (50%)", 0.5);
        velocityMultiplierChooser.setDefaultOption("EGG (30%)", 0.3);

        tuneablesTable.addChild("velocity chooser", velocityMultiplierChooser);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double velocityMultiplier = velocityMultiplierChooser.getSelected();
        double rotationMultiplier = isSensetiveMode.getAsBoolean() ? SwerveContants.SENSETIVE_ROTATION_MULTIPLIER : 1;
        double forwardMultiplier = isSensetiveMode.getAsBoolean() ? SwerveContants.SENSETIVE_FORWARD_MULTIPLIER : 1;

        swerve.drive(
                forwardSupplier.getAsDouble() * MAX_SPEED_MPS * velocityMultiplier * forwardMultiplier,
                sidewaysSupplier.getAsDouble() * MAX_SPEED_MPS * velocityMultiplier * forwardMultiplier,
                rotationsSupplier.getAsDouble() * maxSpeedAngular.get() * velocityMultiplier * rotationMultiplier,
                isFieldRelative.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        tuneablesTable.initTuneable(builder);
    }
}
