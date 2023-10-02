package frc.robot.subsystems.arm.io;
import java.util.function.Supplier;

import frc.robot.Utils.fields.FieldsTable;
import frc.robot.Utils.fields.IOBase;

public abstract class ArmIO extends IOBase{
    public final Supplier<Double> shoulderAngle = fields.addDouble("shoulder angle", this::getShoulderAbsoluteAngle);
    public final Supplier<Double> elbowAngle = fields.addDouble("elbow angle", this::getElbowAbsoluteAngle);
    
    public ArmIO(FieldsTable fieldsTable) {
        super(fieldsTable);
    }

    
    //-----INPUTS---------------------
    protected abstract double getElbowAbsoluteAngle();

    protected abstract double getShoulderAbsoluteAngle();


    //-----OUTPUTS--------------------
    public abstract void setVoltageShoulder(double demand);

    public abstract void setVoltageElbow(double demand);





}
