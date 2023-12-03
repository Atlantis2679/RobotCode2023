package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

import static frc.robot.Constants.Autos.GetOnChargeStationConstants.*;

public class GetOnChargeStation extends Command {
    private final Drivetrain drivetrain;
    private final boolean isReversed;

    public GetOnChargeStation(Drivetrain drivetrain, boolean isReversed) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        this.isReversed = isReversed;
    }

    @Override
    public void initialize() {
        double driveSpeed = (isReversed ? -1 : 1) * DRIVE_SPEED;

        drivetrain.setSpeed(driveSpeed, driveSpeed);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setSpeed(0, 0);
    }

    @Override
    public boolean isFinished() {
        return (isReversed ? -1 : 1) * drivetrain.getPitch() > FINISH_ANGLE;
    }

}
