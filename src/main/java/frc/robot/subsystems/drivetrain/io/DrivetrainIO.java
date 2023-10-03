package frc.robot.subsystems.drivetrain.io;

import java.util.function.Supplier;

import frc.robot.Utils.fields.FieldsTable;
import frc.robot.Utils.fields.IOBase;

public abstract class DrivetrainIO extends IOBase {
    public final Supplier<Double> leftDistanceMeters = fields.addDouble("leftDistanceMeters", this::getLeftDistanceMeters);
    public final Supplier<Double> rightDistanceMeters = fields.addDouble("rightDistanceMeters", this::getRightDistanceMeters);
    public final Supplier<Double> yaw = fields.addDouble("yaw", this::getYaw);
    public final Supplier<Double> pitch = fields.addDouble("pitch", this::getPitch);

    public DrivetrainIO(FieldsTable fieldsTable) {
        super(fieldsTable);
    }

    
    //-----INPUTS---------------------
    protected abstract double getPitch();
    
    protected abstract double getYaw();
    
    protected abstract double getLeftDistanceMeters();
    
    protected abstract double getRightDistanceMeters();
    
    //-----OUTPUTS--------------------
    public abstract void setLeftSpeed(double demand);

    public abstract void setRightSpeed(double demand);
    
    public abstract void resetLeftDistance();

    public abstract void resetRightDistance();
    
    public abstract void resetIMU();
}
