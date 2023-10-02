package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.fields.FieldsTable;
import frc.robot.subsystems.arm.io.ArmIO;
import frc.robot.subsystems.arm.io.ArmIOSparkMax;

import static frc.robot.subsystems.arm.ArmConstants.*;

public class Arm extends SubsystemBase {
    private final FieldsTable fields = new FieldsTable(getName());
    private final ArmIO io = new ArmIOSparkMax(fields);

    private final ArmVisualizer armVisualizer = new ArmVisualizer(fields);

    private double encoderShoulderZeroAngle = ENCODER_SHOULDER_ZERO_ANGLE_DEFAULT;
    private double encoderElbowZeroAngle = ENCODER_ELBOW_ZERO_ANGLE_DEFAULT;

    private double shoulderAngle;
    private double elbowAngle;

    public enum LockedStates {
        UNLOCKED,
        LOCKED,
        BADLY_CLOSED_OUT_OF_LOCK;

        public static LockedStates getFromAngles(double shoulderAngle, double elbowAngleRelativeToShoulder) {
            if (shoulderAngle <= LOCKED_MAX_SHOULDER_ANGLE) {
                if (elbowAngleRelativeToShoulder >= LOCKED_MIN_ELBOW_ANGLE) {
                    return LockedStates.LOCKED;
                }
                return LockedStates.BADLY_CLOSED_OUT_OF_LOCK;
            }
            return LockedStates.UNLOCKED;
        }
    }

    private LockedStates lockedState;

    private final ArmFeedforward feedForwardShoulder = new ArmFeedforward(
            Feedforward.Shoulder.KS,
            Feedforward.Shoulder.KG,
            Feedforward.Shoulder.KV,
            Feedforward.Shoulder.KA);

    private final ArmFeedforward feedforwardElbow = new ArmFeedforward(
            Feedforward.Elbow.KS,
            Feedforward.Elbow.KG,
            Feedforward.Elbow.KV,
            Feedforward.Elbow.KA);

    private final PIDController pidControllerShoulder = new PIDController(
            Feedforward.Shoulder.KP,
            Feedforward.Shoulder.KI,
            Feedforward.Shoulder.KD);

    private final PIDController pidControllerElbow = new PIDController(
            Feedforward.Elbow.KP,
            Feedforward.Elbow.KI,
            Feedforward.Elbow.KD);

    private boolean isEmergencyMode = false;

    @Override
    public void periodic() {
        shoulderAngle = normalizeAbsoluteAngle(
                -io.shoulderAngle.get(),
                encoderShoulderZeroAngle,
                ENCODER_MAX_POSITIVE_SHOULDER);

        elbowAngle = normalizeAbsoluteAngle(
                io.elbowAngle.get(),
                encoderElbowZeroAngle,
                ENCODER_MAX_POSITIVE_ELBOW);

        lockedState = LockedStates.getFromAngles(shoulderAngle, elbowAngle);

        armVisualizer.update(getShoulderAngle(), getElbowAngle());

        fields.recordOutput("shoulderAngle", getShoulderAngle());
        fields.recordOutput("elbowAngle", getElbowAngle());

        SmartDashboard.putNumber("arm angle shoulder", getShoulderAngle());
        SmartDashboard.putNumber("arm angle elbow", getElbowAngle());
        SmartDashboard.putNumber("arm shoulder zero angle", encoderShoulderZeroAngle);
        SmartDashboard.putNumber("arm elbow zero angle", encoderElbowZeroAngle);
        SmartDashboard.putBoolean("arm is emeregency mode", isEmergencyMode);
        SmartDashboard.putBoolean("arm is badly closed out of lock", lockedState == LockedStates.BADLY_CLOSED_OUT_OF_LOCK);
        SmartDashboard.putBoolean("arm is shoulder pid at setpoint", shoulderPIDAtSetpoint());
        SmartDashboard.putBoolean("arm is elbow pid at setpoint", elbowPIDAtSetpoint());
    }

    public void resetEncoders() {
        encoderShoulderZeroAngle = -io.shoulderAngle.get() * 360;
        encoderElbowZeroAngle = io.elbowAngle.get() * 360;
    }

    public void setVoltageShoulder(double demand) {
        SmartDashboard.putNumber("shoulder voltage demand", demand);

        if (lockedState == LockedStates.BADLY_CLOSED_OUT_OF_LOCK && demand < 0)
            demand = 0;
        io.setVoltageShoulder(MathUtil.clamp(demand, -12 * SPEED_LIMIT_SHOULDER, 12 * SPEED_LIMIT_SHOULDER));
    }

    public void setVoltageElbow(double demand) {
        SmartDashboard.putNumber("elbow voltage demand", demand);

        if (lockedState == LockedStates.LOCKED && demand < 0)
            demand = 0;

        io.setVoltageElbow(MathUtil.clamp(demand, -12 * SPEED_LIMIT_ELBOW, 12 * SPEED_LIMIT_ELBOW));
    }

    public double normalizeAbsoluteAngle(double angle, double zeroAngle, double maxPositive) {
        double maxNegative = maxPositive - 360;

        angle *= 360;
        angle -= zeroAngle;
        if (angle > maxPositive)
            angle -= 360;
        if (angle < maxNegative)
            angle += 360;

        return angle;
    }

    public double getShoulderAngle() {
        return shoulderAngle;
    }

    public double getElbowAngle() {
        return elbowAngle;
    }

    public LockedStates getLockedState() {
        return lockedState;
    }

    public ArmValues<Double> calculateFeedforward(
            double shoulderAngle,
            double elbowAngle,
            double shoulderVelocity,
            double elbowVelocity,
            boolean usePID) {

        if (isEmergencyMode)
            return new ArmValues<Double>(0.0, 0.0);

        ArmValues<Double> voltages = new ArmValues<>(
                feedForwardShoulder.calculate(Math.toRadians(shoulderAngle), shoulderVelocity),
                feedforwardElbow.calculate(Math.toRadians(elbowAngle + shoulderAngle),
                        elbowVelocity));

        if (usePID) {
            voltages.shoulder += pidControllerShoulder.calculate(this.shoulderAngle, shoulderAngle);
            voltages.elbow += pidControllerElbow.calculate(this.elbowAngle, elbowAngle);
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
}
