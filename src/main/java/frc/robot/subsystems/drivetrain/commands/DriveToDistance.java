package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.DriveToDistance.*;

public class DriveToDistance extends CommandBase {
    private final Drivetrain drivetrain;
    private final PIDController pidControllerLeft = new PIDController(KP, KI, KD);
    private final PIDController pidControllerRight = new PIDController(KP, KI, KD);

    public DriveToDistance(Drivetrain drivetrain, double meters) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        pidControllerLeft.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
        pidControllerLeft.setSetpoint(meters);
        pidControllerRight.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
        pidControllerRight.setSetpoint(meters);
    }

    @Override
    public void initialize() {
        drivetrain.resetEncoders();
        pidControllerLeft.reset();
        pidControllerRight.reset();
    }

    @Override
    public void execute() {
        double pidResultLeft = pidControllerLeft.calculate(drivetrain.getLeftDistanceMeters());
        double pidResultRight = pidControllerRight.calculate(drivetrain.getRightDistanceMeters());

        drivetrain.setSpeed(pidResultLeft, pidResultRight);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setSpeed(0, 0);
    }

    @Override
    public boolean isFinished() {
        return pidControllerLeft.atSetpoint() && pidControllerRight.atSetpoint();
    }
}
