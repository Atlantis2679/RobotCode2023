package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
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

    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(0),
            0,
            0,
            new Pose2d(new Translation2d(8, 3), Rotation2d.fromDegrees(0)));

    public Drivetrain() {
        io.resetIMU();
    }

    public void setSpeed(double leftDemand, double rightDemand) {
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

    public Rotation2d getRotation2d() {
        return new Rotation2d(Math.toRadians(getYaw()));
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                getRotation2d(),
                getLeftDistanceMeters(),
                getRightDistanceMeters(),
                pose);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IMU pitch", getPitch());
        SmartDashboard.putNumber("left encoder drivetrain", getLeftDistanceMeters());
        SmartDashboard.putNumber("right encoder drivetrain", getRightDistanceMeters());

        odometry.update(getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());

        fields.recordOutput("Odometry", odometry.getPoseMeters());
    }
}
