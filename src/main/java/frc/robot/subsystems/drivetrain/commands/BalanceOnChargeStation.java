package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class BalanceOnChargeStation extends CommandBase {
  private final Drivetrain drivetrain;
  private final double POSITION_TOLERANCE = Constants.Autos.BalanceOnChargeStationAuto.POSITION_TOLERANCE;
  private final double VELOCITY_TOLERANCE = Constants.Autos.BalanceOnChargeStationAuto.VELOCITY_TOLERANCE;
  private final double TARGET_ANGLE = Constants.Autos.BalanceOnChargeStationAuto.TARGET_ANGLE;
  private final boolean IS_REVERSED = Constants.Autos.BalanceOnChargeStationAuto.IS_REVERSED;


  private final PIDController pidController = new PIDController(
      Constants.Autos.BalanceOnChargeStationAuto.KP,
      Constants.Autos.BalanceOnChargeStationAuto.KI,
      Constants.Autos.BalanceOnChargeStationAuto.KD
  );

  public BalanceOnChargeStation(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    pidController.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
    pidController.setSetpoint(TARGET_ANGLE);
  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
    double pitch = (IS_REVERSED ? -1 : 1) * drivetrain.getPitch();
    double pidResult = pidController.calculate(pitch);
    drivetrain.setSpeed(pidResult, pidResult);
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
