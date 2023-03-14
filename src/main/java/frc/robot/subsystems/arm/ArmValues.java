package frc.robot.subsystems.arm;

public class ArmValues<T> {
    public T shoulder;
    public T elbow;

    public ArmValues(T shoulderValue, T elbowValue) {
        shoulder = shoulderValue;
        elbow = elbowValue;
    }
}
