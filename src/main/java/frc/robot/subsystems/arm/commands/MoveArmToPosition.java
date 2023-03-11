package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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

    private boolean shouldMoveShoulder = true;
    private boolean shouldMoveElbow = true;

    private final Timer timer = new Timer();

    public enum Joint {
        SHOULDER,
        ELBOW
    }

    public MoveArmToPosition(Arm arm, double targetPosition, Joint joint) {
        this(arm,
                joint == Joint.SHOULDER ? targetPosition : 0,
                joint == Joint.ELBOW ? targetPosition : 0);

        if (joint == Joint.ELBOW) {
            shouldMoveShoulder = false;
        } else {
            shouldMoveElbow = false;
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
        timer.restart();
        arm.resetPIDs();

        if (shouldMoveShoulder)
            trapezoidProfileShoulder = new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                            ArmConstants.Feedforward.Shoulder.MAX_VELOCITY,
                            ArmConstants.Feedforward.Shoulder.MAX_ACCELERATION),
                    new TrapezoidProfile.State(targetPositionShoulder, 0),
                    new TrapezoidProfile.State(arm.getShoulderAngle(), 0));

        if (shouldMoveElbow)
            trapezoidProfileElbow = new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                            ArmConstants.Feedforward.Elbow.MAX_VELOCITY,
                            ArmConstants.Feedforward.Elbow.MAX_ACCELERATION),
                    new TrapezoidProfile.State(targetPositionElbow, 0),
                    new TrapezoidProfile.State(arm.getElbowAngle(), 0));
    }

    @Override
    public void execute() {
        TrapezoidProfile.State setpointsShoulder = shouldMoveShoulder
                ? trapezoidProfileShoulder.calculate(timer.get())
                : new TrapezoidProfile.State(arm.getShoulderAngle(), 0);

        TrapezoidProfile.State setpointsElbow = shouldMoveElbow
                ? trapezoidProfileElbow.calculate(timer.get())
                : new TrapezoidProfile.State(arm.getElbowAngle(), 0);

        ArmValues<Double> feedforwardResults = arm.calculateFeedforward(
                setpointsShoulder.position,
                setpointsElbow.position,
                setpointsShoulder.velocity,
                setpointsElbow.velocity,
                true);


        arm.setVoltageShoulder(feedforwardResults.shoulder);
        arm.setVoltageElbow(feedforwardResults.elbow);

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return arm.pidsAtSetpoints();
    }
}
