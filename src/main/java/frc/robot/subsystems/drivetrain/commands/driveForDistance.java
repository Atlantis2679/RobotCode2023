// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
public class driveForDistance extends CommandBase {
  /** Creates a new driveFOrDistance. */
  private Drivetrain drivetrain;
  private double metersDistance;
  private double speed;
  private boolean isReversed;
  private double startLeft = 0;
  private double startRight = 0;
final double speedDemand = speed * (isReversed ? -1 : 1);
  public driveForDistance(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = drivetrain;
       }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //drivetrain.resetEncoders();
    startLeft = drivetrain.getLeftDistanceMeters();
    startRight = drivetrain.getRightDistanceMeters();
    drivetrain.setSpeed(speedDemand, speedDemand);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isReversed) {
      return (drivetrain.getLeftDistanceMeters()-startLeft) < -metersDistance
              && (drivetrain.getRightDistanceMeters()-startRight) < -metersDistance;
  }

  return (drivetrain.getLeftDistanceMeters()-startLeft) > metersDistance
          && (drivetrain.getRightDistanceMeters()-startRight) > metersDistance;
  }
}
