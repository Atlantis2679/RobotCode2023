package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmValues;

public class MoveArmToPosition extends CommandBase {
    private final Arm arm;

    private TrapezoidProfile trapezoidProfileShoulder;
    private TrapezoidProfile trapezoidProfileElbow;

    private double targetPositionShoulder;
    private double targetPositionElbow;

    private boolean isElbowMoving = false;
    private boolean isShoulderMoving = false;

    private PassLockDirection passLockDirection;

    private final Timer shoulderTimer = new Timer();
    private final Timer elbowTimer = new Timer();

    public enum PassLockDirection {
        UNLOCKING,
        LOCKING,
        NONE;

        public static PassLockDirection getFromShoulderAngles(double initialAngle, double goalAngle) {
            if (initialAngle < ArmConstants.LOCKED_MAX_SHOULDER_ANGLE
                    && goalAngle > ArmConstants.LOCKED_MAX_SHOULDER_ANGLE)
                return UNLOCKING;
            if (initialAngle > ArmConstants.LOCKED_MAX_SHOULDER_ANGLE
                    && goalAngle < ArmConstants.LOCKED_MAX_SHOULDER_ANGLE)
                return LOCKING;
            return NONE;
        }
    }

    public MoveArmToPosition(Arm arm, double targetPositionShoulder, double targetPositionElbow) {
        this.arm = arm;
        addRequirements(arm);

        this.targetPositionShoulder = targetPositionShoulder;
        this.targetPositionElbow = targetPositionElbow;
    }

    @Override
    public void initialize() {
        passLockDirection = PassLockDirection.getFromShoulderAngles(arm.getShoulderAngle(), targetPositionShoulder);
        if (passLockDirection == PassLockDirection.LOCKING) {
            elbowTimer.restart();
            isShoulderMoving = false;
            isElbowMoving = true;
        } else if (passLockDirection == PassLockDirection.UNLOCKING) {
            shoulderTimer.restart();
            isShoulderMoving = true;
            isElbowMoving = false;
        } else {
            shoulderTimer.restart();
            elbowTimer.restart();
            isShoulderMoving = true;
            isElbowMoving = true;
        }
        arm.resetPIDs();

        trapezoidProfileShoulder = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        ArmConstants.Feedforward.Shoulder.MAX_VELOCITY,
                        ArmConstants.Feedforward.Shoulder.MAX_ACCELERATION),
                new TrapezoidProfile.State(targetPositionShoulder, 0),
                new TrapezoidProfile.State(arm.getShoulderAngle(), 0));

        trapezoidProfileElbow = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        ArmConstants.Feedforward.Elbow.MAX_VELOCITY,
                        ArmConstants.Feedforward.Elbow.MAX_ACCELERATION),
                new TrapezoidProfile.State(targetPositionElbow, 0),
                new TrapezoidProfile.State(arm.getElbowAngle(), 0));
    }

    @Override
    public void execute() {
        if (passLockDirection == PassLockDirection.LOCKING
                && !isShoulderMoving
                && ((trapezoidProfileElbow.totalTime() + 0.3) - elbowTimer.get()) <= trapezoidProfileShoulder
                        .timeLeftUntil(ArmConstants.LOCKED_MAX_SHOULDER_ANGLE)) {

            shoulderTimer.restart();
            isShoulderMoving = true;
        }

        if (passLockDirection == PassLockDirection.UNLOCKING
                && !isElbowMoving
                && arm.getLockedState() == Arm.LockedStates.UNLOCKED) {

            elbowTimer.restart();
            isElbowMoving = true;
        }

        TrapezoidProfile.State setpointsShoulder = getShoulderSetpoints();
        TrapezoidProfile.State setpointsElbow = getElbowSetpoints();

        ArmValues<Double> feedforwardResults = arm.calculateFeedforward(
                setpointsShoulder.position,
                setpointsElbow.position,
                setpointsShoulder.velocity,
                setpointsElbow.velocity,
                true);

        arm.setVoltageShoulder(feedforwardResults.shoulder);
        arm.setVoltageElbow(feedforwardResults.elbow);
    }

    private TrapezoidProfile.State getShoulderSetpoints() {
        if (!isShoulderMoving)
            return new TrapezoidProfile.State(arm.getShoulderAngle(), 0);
        return trapezoidProfileShoulder.calculate(shoulderTimer.get());
    }

    private TrapezoidProfile.State getElbowSetpoints() {
        if (arm.getLockedState() == Arm.LockedStates.LOCKED)
            return new TrapezoidProfile.State(ArmConstants.ANGLE_REST_ELBOW, 0);
        if (!isElbowMoving)
            return new TrapezoidProfile.State(arm.getElbowAngle(), 0);
        return trapezoidProfileElbow.calculate(elbowTimer.get());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return trapezoidProfileShoulder.isFinished(shoulderTimer.get()) && arm.shoulderPIDAtSetpoint()
                && trapezoidProfileElbow.isFinished(elbowTimer.get()) && arm.elbowPIDAtSetpoint();
    }
}
