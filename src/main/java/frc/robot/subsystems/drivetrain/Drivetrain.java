package frc.robot.subsystems.drivetrain;



import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import frc.robot.Utils.fields.FieldsTable;
import frc.robot.subsystems.drivetrain.io.DrivetrainIO;
import frc.robot.subsystems.drivetrain.io.DrivetrainIOTalon;

public class Drivetrain extends SubsystemBase {
    private final FieldsTable fields = new FieldsTable(getName());
    private final DrivetrainIO io = new DrivetrainIOTalon(fields);
    private double pitchOffset = PITCH_OFFSET_DEFAULT;



    public void setSpeed(double leftDemand, double rightDemand){
        leftDemand = MathUtil.clamp(leftDemand, -1, 1);
        rightDemand = MathUtil.clamp(rightDemand, -1, 1);
        io.setLeftSpeed(leftDemand);
        io.setRightSpeed(rightDemand);
    }

    public double getPitch() {
        return io.pitch.get() - pitchOffset;
    }

    public void resetPitch() {
        pitchOffset = io.pitch.get();
    }

    public void resetEncoders() {
        io.resetLeftDistance();
        io.resetRightDistance();
    }

    public double getYaw() {
        return io.yaw.get();
    }

    public void setYaw(double angle) {
        io.setYaw(angle);
    }

    public double getLeftDistanceMeters() {
        return io.leftDistanceMeters.get();
    }

    public double getRightDistanceMeters() {
        return io.rightDistanceMeters.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IMU pitch", getPitch());
        SmartDashboard.putNumber("left encoder drivetrain", getLeftDistanceMeters());
        SmartDashboard.putNumber("right encoder drivetrain", getRightDistanceMeters());
    }
}
