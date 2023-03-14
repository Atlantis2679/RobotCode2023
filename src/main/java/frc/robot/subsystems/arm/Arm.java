package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static Arm instance = null;
    private final CANSparkMax motorShoulder = new CANSparkMax(ArmConstants.MOTOR_SHOULDER_ID, MotorType.kBrushless);
    private final CANSparkMax motorShoulderFollower = new CANSparkMax(ArmConstants.MOTOR_SHOULDER_FOLLOWER_ID,
            MotorType.kBrushless);
    private final CANSparkMax motorElbow = new CANSparkMax(ArmConstants.MOTOR_ELBOW_ID, MotorType.kBrushless);

    private final DutyCycleEncoder encoderShoulder = new DutyCycleEncoder(ArmConstants.ENCODER_SHOULDER_ID);
    private final DutyCycleEncoder encoderElbow = new DutyCycleEncoder(ArmConstants.ENCODER_ELBOW_ID);
    private final DigitalInput limitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH_ID);
    private final double SPEED_LIMIT_SHOULDER = ArmConstants.SPEED_LIMIT_SHOULDER;
    private final double SPEED_LIMIT_ELBOW = ArmConstants.SPEED_LIMIT_ELBOW;

    private double encoderShoulderZeroAngle = ArmConstants.ENCODER_SHOULDER_ZERO_ANGLE;
    private double ENCODER_MAX_POSITIVE_SHOULDER = ArmConstants.ENCODER_MAX_POSITIVE_SHOULDER;
    private double encoderElbowZeroAngle = ArmConstants.ENCODER_ELBOW_ZERO_ANGLE;
    private double ENCODER_MAX_POSITIVE_ELBOW = ArmConstants.ENCODER_MAX_POSITIVE_ELBOW;

    private final ArmFeedforward feedForwardShoulder = new ArmFeedforward(
            ArmConstants.Feedforward.Shoulder.KS,
            ArmConstants.Feedforward.Shoulder.KG,
            ArmConstants.Feedforward.Shoulder.KV,
            ArmConstants.Feedforward.Shoulder.KA);

    private final ArmFeedforward feedforwardElbow = new ArmFeedforward(
            ArmConstants.Feedforward.Elbow.KS,
            ArmConstants.Feedforward.Elbow.KG,
            ArmConstants.Feedforward.Elbow.KV,
            ArmConstants.Feedforward.Elbow.KA);

    private final PIDController pidControllerShoulder = new PIDController(
            ArmConstants.Feedforward.Shoulder.KP,
            ArmConstants.Feedforward.Shoulder.KI,
            ArmConstants.Feedforward.Shoulder.KD);

    private final PIDController pidControllerElbow = new PIDController(
            ArmConstants.Feedforward.Elbow.KP,
            ArmConstants.Feedforward.Elbow.KI,
            ArmConstants.Feedforward.Elbow.KD);

    private boolean isEmergencyMode = false;

    private Arm() {
        motorShoulder.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT_SHOULDER_AMP);
        motorShoulder.setInverted(true);
        motorShoulderFollower.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT_SHOULDER_AMP);
        motorShoulder.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motorShoulderFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motorShoulderFollower.follow(motorShoulder, true);

        motorElbow.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT_ELBOW_AMP);
        motorElbow.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motorElbow.setInverted(true);

        encoderShoulder.setPositionOffset(0);
        encoderElbow.setPositionOffset(0);

        pidControllerShoulder.setTolerance(
                ArmConstants.Feedforward.Shoulder.TOLERANCE_POSITION,
                ArmConstants.Feedforward.Shoulder.TOLERANCE_VELOCITY);

        pidControllerElbow.setTolerance(
                ArmConstants.Feedforward.Elbow.TOLERANCE_POSITION,
                ArmConstants.Feedforward.Elbow.TOLERANCE_VELOCITY);
    }

    @Override
    public void periodic() {


        SmartDashboard.putNumber("arm angle shoulder", getShoulderAngle());
        SmartDashboard.putNumber("arm angle elbow", getElbowAngle(false));
        SmartDashboard.putNumber("arm angle elbow relative to shoulder", getElbowAngle(true));
        SmartDashboard.putNumber("arm shoulder zero angle", encoderShoulderZeroAngle);
        SmartDashboard.putNumber("arm elbow zero angle", encoderElbowZeroAngle);
        SmartDashboard.putBoolean("arm limit switch", !limitSwitch.get());
        SmartDashboard.putBoolean("arm is emeregency mode", isEmergencyMode);
        SmartDashboard.putBoolean("arm is shoulder pid at setpoint", shoulderPIDAtSetpoint());
        SmartDashboard.putBoolean("arm is elbow pid at setpoint", elbowPIDAtSetpoint());
    }

    public void resetEncoders() {
        encoderShoulderZeroAngle = -encoderShoulder.getAbsolutePosition() * 360;
        encoderElbowZeroAngle = encoderElbow.getAbsolutePosition() * 360;
    }

    public void setSpeedShoulder(double demand) {
        motorShoulder.set(!isEmergencyMode && !limitSwitch.get() && demand < 0
                ? 0
                : MathUtil.clamp(demand, -SPEED_LIMIT_SHOULDER, SPEED_LIMIT_SHOULDER));
    }

    public void setSpeedElbow(double demand) {
        motorElbow.set(MathUtil.clamp(demand, -SPEED_LIMIT_ELBOW, SPEED_LIMIT_ELBOW));
    }

    public void setVoltageShoulder(double demand) {
        SmartDashboard.putNumber("shoulder voltage demand", demand);

        motorShoulder.setVoltage(!isEmergencyMode && !limitSwitch.get() && demand < 0
                ? 0
                : MathUtil.clamp(demand, -12 * SPEED_LIMIT_SHOULDER, 12 * SPEED_LIMIT_SHOULDER));
    }

    public void setVoltageElbow(double demand) {
        SmartDashboard.putNumber("elbow voltage demand", demand);
        motorElbow.setVoltage(MathUtil.clamp(demand, -12 * SPEED_LIMIT_ELBOW, 12 * SPEED_LIMIT_ELBOW));
    }

    public double normalizeAbsoluteAngle(double angle, double zeroAngle, double maxPositive) {
        double maxNegative = maxPositive - 360;

        angle *= 360;
        angle -= zeroAngle;
        if(angle > maxPositive) angle -= 360;
        if(angle < maxNegative) angle += 360;

        return angle;
    }

    public double getShoulderAngle() {
        return normalizeAbsoluteAngle(
                -encoderShoulder.getAbsolutePosition(),
                encoderShoulderZeroAngle,
                ENCODER_MAX_POSITIVE_SHOULDER);
    }

    public double getElbowAngle(boolean isRelativeToShoulder) {
        return normalizeAbsoluteAngle(
                encoderElbow.getAbsolutePosition(),
                encoderElbowZeroAngle,
                ENCODER_MAX_POSITIVE_ELBOW)
                + (isRelativeToShoulder ? 0 : getShoulderAngle());
    }

    public ArmValues<Double> calculateFeedforward(
            double shoulderAngle,
            double elbowAngle,
            double shoulderVelocity,
            double elbowVelocity,
            boolean usePID,
            boolean isRelativeToShoulder) {

        if (isEmergencyMode)
            return new ArmValues<Double>(0.0, 0.0);

        ArmValues<Double> voltages = new ArmValues<>(
                feedForwardShoulder.calculate(Math.toRadians(shoulderAngle), shoulderVelocity),
                feedforwardElbow.calculate(Math.toRadians(elbowAngle + (isRelativeToShoulder ? getShoulderAngle() : 0)), elbowVelocity));

        if (usePID) {
            voltages.shoulder += pidControllerShoulder.calculate(getShoulderAngle(), shoulderAngle);
            voltages.elbow += pidControllerElbow.calculate(getElbowAngle(isRelativeToShoulder), elbowAngle);
        }

        return voltages;
    }

    public void resetPIDs() {
        pidControllerShoulder.reset();
        pidControllerElbow.reset();
    }

    public boolean shoulderPIDAtSetpoint() {
        return pidControllerShoulder.atSetpoint();
    }

    public boolean elbowPIDAtSetpoint() {
        return pidControllerElbow.atSetpoint();
    }

    public void setEmergencyMode(boolean isEmergencyMode) {
        this.isEmergencyMode = isEmergencyMode;
    }

    public boolean getEmergencyMode() {
        return isEmergencyMode;
    }

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }
}
