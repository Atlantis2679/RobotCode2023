package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain drivetrain;

  private final PIDController pid = new PIDController(Constants.ArcadeDrive.KP, Constants.ArcadeDrive.KI, Constants.ArcadeDrive.KD);


  public ArcadeDrive(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // inverted because the Yaxis joystick gives opposite values.
    double xSpeed = -OI.driverController.getLeftY();
    double zRotation = OI.driverController.getRightX();

    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
    zRotation = Math.copySign(zRotation * zRotation, zRotation);

    double leftSpeed;
    double rightSpeed;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (Double.compare(xSpeed, 0.0) >= 0) {
      // First quadrant, else second quadrant
      if (Double.compare(zRotation, 0.0) >= 0) {
        leftSpeed = maxInput;
        rightSpeed = xSpeed - zRotation;
      } else {
        leftSpeed = xSpeed + zRotation;
        rightSpeed = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (Double.compare(zRotation, 0.0) >= 0) {
        leftSpeed = xSpeed + zRotation;
        rightSpeed = maxInput;
      } else {
        leftSpeed = maxInput;
        rightSpeed = xSpeed - zRotation;
      }
    }

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
    if (maxMagnitude > 1.0) {
      leftSpeed /= maxMagnitude;
      rightSpeed /= maxMagnitude;
    }

    // double leftPIDValue = pid.calculate(drivetrain.getLeftEncoder().getRate() / Constants.ArcadeDrive.MAX_SPEED, leftSpeed);
    // double rightPIDValue = pid.calculate(drivetrain.getRightEncoder().getRate() / Constants.ArcadeDrive.MAX_SPEED, rightSpeed);

    // SmartDashboard.putNumber("rightPIDValue", rightPIDValue);
    // SmartDashboard.putNumber("leftPIDValue", leftPIDValue);

    // double PercentOutputLeftValue = MathUtil.clamp(leftPIDValue, -1, 1);
    // double PercentOutputRightValue = MathUtil.clamp(rightPIDValue, -1, 1);
    drivetrain.set(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
