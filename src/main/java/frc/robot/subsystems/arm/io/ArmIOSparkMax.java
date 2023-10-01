package frc.robot.subsystems.arm.io;
import static frc.robot.subsystems.arm.ArmConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Utils.fields.FieldsTable;

public class ArmIOSparkMax extends ArmIO {
    private final CANSparkMax motorShoulder = new CANSparkMax(MOTOR_SHOULDER_ID, MotorType.kBrushless);
    private final CANSparkMax motorShoulderFollower = new CANSparkMax(MOTOR_SHOULDER_FOLLOWER_ID,
            MotorType.kBrushless);
    private final CANSparkMax motorElbow = new CANSparkMax(MOTOR_ELBOW_ID, MotorType.kBrushless);

    private final DutyCycleEncoder encoderShoulder = new DutyCycleEncoder(ENCODER_SHOULDER_ID);
    private final DutyCycleEncoder encoderElbow = new DutyCycleEncoder(ENCODER_ELBOW_ID);

    private double encoderShoulderZeroAngle = ENCODER_SHOULDER_ZERO_ANGLE_DEFAULT;
    private double encoderElbowZeroAngle = ENCODER_ELBOW_ZERO_ANGLE_DEFAULT;

    private double shoulderAngle;
    private double elbowAngle;
    

    public ArmIOSparkMax(FieldsTable fieldsTable) {
        super(fieldsTable);

        motorShoulder.setSmartCurrentLimit(CURRENT_LIMIT_SHOULDER_AMP);
        motorShoulder.setInverted(true);
        motorShoulderFollower.setSmartCurrentLimit(CURRENT_LIMIT_SHOULDER_AMP);
        motorShoulder.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motorShoulderFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motorShoulderFollower.follow(motorShoulder, true);

        motorElbow.setSmartCurrentLimit(CURRENT_LIMIT_ELBOW_AMP);
        motorElbow.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motorElbow.setInverted(true);

        encoderShoulder.setPositionOffset(0);
        encoderElbow.setPositionOffset(0);
    }

    public void resetEncoders() {
        encoderShoulderZeroAngle = -encoderShoulder.getAbsolutePosition() * 360;
        encoderElbowZeroAngle = encoderElbow.getAbsolutePosition() * 360;
    }

    @Override
    public void setVoltageShoulder(double demand) {
        motorShoulder.setVoltage(MathUtil.clamp(demand, -12 * SPEED_LIMIT_SHOULDER, 12 * SPEED_LIMIT_SHOULDER));
    }

    @Override
    public double getShoulderAbsoluteAngle() {
        return encoderShoulder.getAbsolutePosition();
    }

    @Override
    public double getElbowAbsoluteAngle() {
        return encoderElbow.getAbsolutePosition();
    }

    @Override
    public void setVoltageElbow(double demand) {
        motorElbow.setVoltage(demand);
    }


}
