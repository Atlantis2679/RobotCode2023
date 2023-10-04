package frc.robot.subsystems.intake.io;

import frc.robot.Utils.fields.FieldsTable;
import frc.robot.Utils.fields.IOBase;

public abstract class IntakeIO extends IOBase {
    public IntakeIO(FieldsTable fieldsTable) {
        super(fieldsTable);
    }

    public abstract void setSpeed(double demand);
}
