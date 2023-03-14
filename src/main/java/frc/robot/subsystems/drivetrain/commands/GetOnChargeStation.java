package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class GetOnChargeStation extends CommandBase {
    private final Drivetrain drivetrain;
    private final boolean IS_REVERSED = Constants.Autos.BalanceOnChargeStationAuto.IS_REVERSED;
    private final double FINISH_ANGLE = Constants.Autos.GetOnChargeStationAuto.FINISH_ANGLE;

    public GetOnChargeStation(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        double DRIVE_SPEED = Constants.Autos.GetOnChargeStationAuto.DRIVE_SPEED;
        double driveSpeed = (IS_REVERSED ? -1 : 1) * DRIVE_SPEED;

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
        return (IS_REVERSED ? -1 : 1) * drivetrain.getPitch() > FINISH_ANGLE;
    }

}
