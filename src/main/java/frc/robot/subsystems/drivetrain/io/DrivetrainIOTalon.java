package frc.robot.subsystems.drivetrain.io;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Utils.fields.FieldsTable;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.Encoder;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

public class DrivetrainIOTalon extends DrivetrainIO {
    private final TalonSRX leftMotor = new TalonSRX(LEFT_ID);
    private final TalonSRX leftMotorFollower = new TalonSRX(LEFT_FOLLOWER_ID);
    private final TalonSRX rightMotor = new TalonSRX(RIGHT_ID);
    private final TalonSRX rightMotorFollower = new TalonSRX(RIGHT_FOLLOWER_ID);

    private final  WPI_PigeonIMU imu = new WPI_PigeonIMU(rightMotorFollower);
    private final Encoder leftEncoder = new Encoder(LEFT_ENCODER_CHANNEL_A, LEFT_ENCODER_CHANNEL_B);
    private final Encoder rightEncoder = new Encoder(RIGHT_ENCODER_CHANNEL_A, RIGHT_ENCODER_CHANNEL_B);

    public DrivetrainIOTalon(FieldsTable fieldsTable) {
        super(fieldsTable);
        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(
                true,
                CURRENT_LIMIT_AMP,
                0,
                0);
        leftMotor.configSupplyCurrentLimit(currentLimitConfiguration);
        rightMotor.configSupplyCurrentLimit(currentLimitConfiguration);

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

    @Override
    protected double getPitch() {
        return imu.getPitch();
    }

    @Override
    protected double getYaw() {
        return imu.getYaw();
    }

    @Override
    protected double getLeftDistanceMeters() {
        return leftEncoder.getDistance();
    }

    @Override
    protected double getRightDistanceMeters() {
        return rightEncoder.getDistance();
    }

    @Override
    public void setLeftSpeed(double demand) {
        leftMotor.set(ControlMode.PercentOutput, demand);        
    }

    @Override
    public void setRightSpeed(double demand) {
        rightMotor.set(ControlMode.PercentOutput, demand);        
    }

    @Override
    public void resetLeftDistance(){
        leftEncoder.reset();
    }

    @Override
    public void resetRightDistance(){
        rightEncoder.reset();
    }

    @Override
    public void resetIMU() {
        imu.reset();
    }
}
