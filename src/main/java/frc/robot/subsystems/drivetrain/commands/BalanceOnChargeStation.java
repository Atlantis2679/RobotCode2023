package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

import static frc.robot.Constants.Autos.BalanceOnChargeStationConstants.*;

public class BalanceOnChargeStation extends Command {
  private final Drivetrain drivetrain;
  private final PIDController pidController = new PIDController(KP, KI, KD);

  public BalanceOnChargeStation(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    pidController.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
    pidController.setSetpoint(0);
  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
    double pidResult = pidController.calculate(drivetrain.getPitch());
    double speed = MathUtil.clamp(pidResult, -MAX_SPEED, MAX_SPEED);
    drivetrain.setSpeed(speed, speed);
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
