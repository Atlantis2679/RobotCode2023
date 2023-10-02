package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Utils.fields.FieldsTable;

public class ArmVisualizer {
    private final FieldsTable fieldsTable;
    private final Mechanism2d mech2d = new Mechanism2d(3, 1.5);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("ShoulderPivot", 1.5, 0);
    private final MechanismLigament2d tower = mech2dRoot.append(
            new MechanismLigament2d(
                    "Tower",
                    1,
                    90,
                    5,
                    new Color8Bit(139, 0, 0)));

    private final MechanismLigament2d shoulderMech2d = tower.append(
            new MechanismLigament2d(
                    "Shoulder",
                    0.8,
                    0,
                    10,
                    new Color8Bit(64, 224, 208)));

    private final MechanismLigament2d elbowMech2d = shoulderMech2d.append(
            new MechanismLigament2d(
                "Elbow",
                0.4,
                0,
                10,
                new Color8Bit(64, 224, 208)));

    public ArmVisualizer(FieldsTable fieldsTable) {
        this.fieldsTable = fieldsTable;
    }

    public void update(double ShoulderAngle, double elbowAngle) {
        shoulderMech2d.setAngle(ShoulderAngle - 90);
        elbowMech2d.setAngle(elbowAngle);
        fieldsTable.recordOutput("Mechanism2d", mech2d);
    }
}
