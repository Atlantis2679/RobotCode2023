package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.TurnByAngle.*;

public class TurnByDegree extends Command {
    private final Drivetrain drivetrain;
    private final PIDController pidController = new PIDController(KP, KI, KD);
    private double startAngle = 0;

    public TurnByDegree(Drivetrain drivetrain, double angle) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        pidController.setSetpoint(angle / 360);
        pidController.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
    }

    @Override
    public void initialize() {
        startAngle = drivetrain.getYaw();
        pidController.reset();
    }

    @Override
    public void execute() {
        double currRotation = (drivetrain.getYaw() - startAngle) / 360;
        double pidResult = pidController.calculate(currRotation);
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
