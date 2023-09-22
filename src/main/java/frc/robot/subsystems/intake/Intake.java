package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.fields.FieldsTable;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOTalon;

public class Intake extends SubsystemBase {
    private final FieldsTable fields = new FieldsTable(getName());
    private final IntakeIO io = new IntakeIOTalon(fields);

    public Intake() {
    }

    public void setSpeed(double demand) {
        io.setSpeed(MathUtil.clamp(demand, -1, 1));
    }

    @Override
    public void periodic() {
    }
}
