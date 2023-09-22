package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;


public class Drivetrain extends SubsystemBase {
    private final TalonSRX leftMotor = new TalonSRX(LEFT_ID);
    private final TalonSRX leftMotorFollower = new TalonSRX(LEFT_FOLLOWER_ID);
    private final TalonSRX rightMotor = new TalonSRX(RIGHT_ID);
    private final TalonSRX rightMotorFollower = new TalonSRX(RIGHT_FOLLOWER_ID);

    private final PigeonIMU imu = new PigeonIMU(rightMotorFollower);
    private final Encoder leftEncoder = new Encoder(LEFT_ENCODER_CHANNEL_A, LEFT_ENCODER_CHANNEL_B);
    private final Encoder rightEncoder = new Encoder(RIGHT_ENCODER_CHANNEL_A, RIGHT_ENCODER_CHANNEL_B);

    private double pitchOffset = PITCH_OFFSET_DEFAULT;

    public Drivetrain() {
        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(
                true,
                DrivetrainConstants.CURRENT_LIMIT_AMP,
                0,
                0
        );
        rightMotor.configSupplyCurrentLimit(currentLimitConfiguration);
        rightMotorFollower.configSupplyCurrentLimit(currentLimitConfiguration);
        leftMotor.configSupplyCurrentLimit(currentLimitConfiguration);
        leftMotorFollower.configSupplyCurrentLimit(currentLimitConfiguration);

        leftMotorFollower.follow(leftMotor);
        rightMotorFollower.follow(rightMotor);

        rightMotor.setInverted(true);
        rightMotorFollower.setInverted(true);

        rightMotor.setNeutralMode(NeutralMode.Brake);
        rightMotorFollower.setNeutralMode(NeutralMode.Brake);
        leftMotor.setNeutralMode(NeutralMode.Brake);
        leftMotorFollower.setNeutralMode(NeutralMode.Brake);

        rightEncoder.setReverseDirection(true);

        double wheelRadiusInMeters = 0.076;
        int pulsesInRound = 2048;

        double distancePerRound = wheelRadiusInMeters * 2 * Math.PI;
        double roundsPerPules = 1.0 / pulsesInRound;

        double distancePerPules = distancePerRound * roundsPerPules;    

        rightEncoder.setDistancePerPulse(distancePerPules);
        leftEncoder.setDistancePerPulse(distancePerPules);
    }

    public void setSpeed(double leftDemand, double rightDemand){
        leftDemand = MathUtil.clamp(leftDemand, -1, 1);
        rightDemand = MathUtil.clamp(rightDemand, -1, 1);
        leftMotor.set(ControlMode.PercentOutput, leftDemand);
        rightMotor.set(ControlMode.PercentOutput, rightDemand);
    }

    public double getPitch() {
        return imu.getPitch() - pitchOffset;
    }

    public void resetPitch() {
        pitchOffset = imu.getPitch();
    }

    public void resetEncoders() {
        rightEncoder.reset();
        leftEncoder.reset();
    }

    public double getYaw() {
        return imu.getYaw();
    }

    public void setYaw(double angle) {
        imu.setYaw(angle);
    }

    public double getLeftDistanceMeters() {
        return leftEncoder.getDistance();
    }

    public double getRightDistanceMeters() {
        return rightEncoder.getDistance();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IMU pitch", getPitch());
        SmartDashboard.putNumber("left encoder drivetrain", getLeftDistanceMeters());
        SmartDashboard.putNumber("right encoder drivetrain", getRightDistanceMeters());
    }
}
