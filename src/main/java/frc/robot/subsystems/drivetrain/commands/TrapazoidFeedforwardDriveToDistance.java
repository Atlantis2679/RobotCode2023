// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.commands;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class TrapazoidFeedforwardDriveToDistance extends CommandBase {
  /** Creates a new TrapazoidFeedforwardDriveToDistance. */
  private final Drivetrain drivetrain;
  private TrapezoidProfile trapezoidProfilePositions;
  private final Timer timer = new Timer();
  private double lastVelocity = 0;
  private double lastTime = 0;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    0, 4.7,0);

  public TrapazoidFeedforwardDriveToDistance(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

    trapezoidProfilePositions = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
              1,
              1),
      new TrapezoidProfile.State(1, 0),
      new TrapezoidProfile.State(0, 0));
      timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("timer trapezoid", timer.get());
    TrapezoidProfile.State state = trapezoidProfilePositions.calculate(timer.get());
    SmartDashboard.putNumber("trapezoid position", state.position);
    SmartDashboard.putNumber("trapezoid velocity", state.velocity);
    double feedforwardVoltage = feedforward.calculate(state.velocity, (state.velocity - lastVelocity)/(timer.get() - lastTime));
    SmartDashboard.putNumber("feedforwardVoltage", feedforwardVoltage);
    drivetrain.setVoltage(feedforwardVoltage, feedforwardVoltage);
    lastTime = timer.get();
    lastVelocity = state.velocity;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trapezoidProfilePositions.isFinished(timer.get());
  }
}
