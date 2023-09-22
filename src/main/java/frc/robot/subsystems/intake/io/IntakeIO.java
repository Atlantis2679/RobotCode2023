package frc.robot.subsystems.intake.io;

import java.util.function.Supplier;

import frc.robot.Utils.fields.FieldsTable;

public abstract class IntakeIO {
    private final FieldsTable fields;

    public IntakeIO(FieldsTable fieldsTable) {
        fields = fieldsTable;
    }

    public abstract void setSpeed(double demand);
}
