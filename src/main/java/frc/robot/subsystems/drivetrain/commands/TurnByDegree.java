package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;

public class TurnByDegree extends CommandBase {
    private final Drivetrain drivetrain;
    private final PIDController pidController = new PIDController(
            DrivetrainConstants.TurnByAngle.KP,
            DrivetrainConstants.TurnByAngle.KI,
            DrivetrainConstants.TurnByAngle.KD);

    public TurnByDegree(Drivetrain drivetrain, double angle) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        pidController.setSetpoint(angle / 360);
        pidController.setTolerance(
                DrivetrainConstants.TurnByAngle.POSITION_TOLERANCE,
                DrivetrainConstants.TurnByAngle.VELOCITY_TOLERANCE
        );
    }

    @Override
    public void initialize() {
        drivetrain.setYaw(0);
        pidController.reset();
    }

    @Override
    public void execute() {
        double currAngle = drivetrain.getYaw() / 360;
        double pidResult = pidController.calculate(currAngle);
        drivetrain.setSpeed(-pidResult, pidResult);

        SmartDashboard.putNumber("degrees yaw", drivetrain.getYaw());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setSpeed(0, 0);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
