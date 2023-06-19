// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.commands;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.AutoPath;

public class PathPlannerAutonomous extends CommandBase {
  /** Creates a new PathPlannerAutonomous. */
  public PathPlannerAutonomous(Drivetrain drivetrain) {
    final DifferentialDriveKinematics K_KINEMATICS = new DifferentialDriveKinematics(AutoPath.WHEEL_DISTANCE);

    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            AutoPath.KS,
            AutoPath.KV,
            AutoPath.KA),
        new DifferentialDriveKinematics(AutoPath.WHEEL_DISTANCE),
        10);

    TrajectoryConfig config = new TrajectoryConfig(
        AutoPath.MAX_VELOCITY,
        AutoPath.MAX_ACCELERATION)
        .setKinematics(K_KINEMATICS)
        .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),

        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),

        new Pose2d(3, 0, new Rotation2d(0)),

        config);
    
      drivetrain.resetOdometry(exampleTrajectory.getInitialPose());  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
