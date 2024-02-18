package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.logfields.LogFieldsTable;

public class IntakeVisualizer {
    private final LogFieldsTable fieldsTable;
    private final MechanismLigament2d wrist;
    private final Mechanism2d mech2d;
    private final MechanismRoot2d root;
    private final String name;

    public IntakeVisualizer(LogFieldsTable fieldsTable, Color8Bit color, String name) {
        this.name = name;
        this.fieldsTable = fieldsTable;

        mech2d = new Mechanism2d(1, 1);

        root = mech2d.getRoot("IntakeAnchorPoint", 0.5, 0.5);

        wrist = root.append(
                new MechanismLigament2d(null, 0.35, 0, 10, color));
    }

    public void update(Double angleDegrees) {
        wrist.setAngle(angleDegrees);
        fieldsTable.recordOutput(name, mech2d);
    }

}
