package frc.robot.subsystems.drivetrain.commands;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.ArcadeDrive.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;

public class ArcadeDrive extends CommandBase {
    private final Drivetrain drivetrain;

    private final DoubleSupplier forwardDemandSupplier;
    private final DoubleSupplier rotationDemandSupplier;
    private final BooleanSupplier IsSensitiveForwardSupplier;
    private final BooleanSupplier IsSensitiveRotationSupplier;

    public ArcadeDrive(
            Drivetrain drivetrain,
            DoubleSupplier forwardDemandSupplier,
            DoubleSupplier rotationDemandSupplier,
            BooleanSupplier isSensitiveForwardSupplier,
            BooleanSupplier isSensitiveRotationSupplier
    ) {
        this.drivetrain = drivetrain;
        addRequirements(this.drivetrain);
        this.forwardDemandSupplier = forwardDemandSupplier;
        this.rotationDemandSupplier = rotationDemandSupplier;
        this.IsSensitiveForwardSupplier = isSensitiveForwardSupplier;
        this.IsSensitiveRotationSupplier = isSensitiveRotationSupplier;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double forwardDemand = forwardDemandSupplier.getAsDouble();
        double rotationDemand = rotationDemandSupplier.getAsDouble();

        forwardDemand = MathUtil.applyDeadband(forwardDemand, Constants.OI.JOYSTICKS_DEADBAND_VALUE);
        rotationDemand = MathUtil.applyDeadband(rotationDemand, Constants.OI.JOYSTICKS_DEADBAND_VALUE);
        
        forwardDemand = MathUtil.clamp(forwardDemand, -1.0, 1.0);
        rotationDemand = MathUtil.clamp(rotationDemand, -1.0, 1.0);

        forwardDemand *= !IsSensitiveForwardSupplier.getAsBoolean() ? SENSITIVE_FORWARD_MULTIPLIER : FORWARD_MULTIPLIER;
        rotationDemand *= !IsSensitiveRotationSupplier.getAsBoolean() ? SENSITIVE_ROTATION_MULTIPLIER : ROTATION_MULTIPLIER;

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        forwardDemand = Math.copySign(forwardDemand * forwardDemand, forwardDemand);
        rotationDemand = Math.copySign(rotationDemand * rotationDemand, rotationDemand);

        double leftSpeed;
        double rightSpeed;
        

        double maxInput = Math.copySign(Math.max(Math.abs(forwardDemand), Math.abs(rotationDemand)), forwardDemand);

        if (Double.compare(forwardDemand, 0.0) >= 0) {
            // First quadrant, else second quadrant
            if (Double.compare(rotationDemand, 0.0) >= 0) {
                leftSpeed = maxInput;
                rightSpeed = forwardDemand - rotationDemand;
            } else {
                leftSpeed = forwardDemand + rotationDemand;
                rightSpeed = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (Double.compare(rotationDemand, 0.0) >= 0) {
                leftSpeed = forwardDemand + rotationDemand;
                rightSpeed = maxInput;
            } else {
                leftSpeed = maxInput;
                rightSpeed = forwardDemand - rotationDemand;
            }
        }

        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (maxMagnitude > 1.0) {
            leftSpeed /= maxMagnitude;
            rightSpeed /= maxMagnitude;
        }

        drivetrain.setSpeed(leftSpeed, rightSpeed);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
