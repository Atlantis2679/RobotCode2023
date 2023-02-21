// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class BalanceOnChargeStationAuto extends CommandBase {
  Drivetrain drivetrain;

  /** Creates a new BalanceOnChargeStationAuto. */
  public BalanceOnChargeStationAuto(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(drivetrain.getPitch() > Constants.BalanceOnChargeStationAuto.MISTAKE_ANGLE)
      drivetrain.set(Constants.BalanceOnChargeStationAuto.DRIVE_SPEED_FORWARD, Constants.BalanceOnChargeStationAuto.DRIVE_SPEED_FORWARD);
    else if(drivetrain.getPitch() < Constants.BalanceOnChargeStationAuto.MISTAKE_ANGLE)
      drivetrain.set(Constants.BalanceOnChargeStationAuto.DRIVE_SPEED_BACKWARDS, Constants.BalanceOnChargeStationAuto.DRIVE_SPEED_BACKWARDS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.set(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getPitch() < Math.abs(Constants.BalanceOnChargeStationAuto.FINISH_ANGLE);
  }
}