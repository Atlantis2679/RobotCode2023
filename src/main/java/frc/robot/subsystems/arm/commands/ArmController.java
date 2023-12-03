package frc.robot.subsystems.arm.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmValues;

public class ArmController extends Command {
  private final Arm arm;

  private final DoubleSupplier shoulderDemandSupplier;
  private final DoubleSupplier elbowDemandSupplier;
  private boolean shouldCleanValues;

  public ArmController(Arm arm, DoubleSupplier shoulderDemandSupplier, DoubleSupplier elbowDemandSupplier, boolean shouldCleanValues) {
    this.arm = arm;
    addRequirements(arm);
    this.shoulderDemandSupplier = shoulderDemandSupplier;
    this.elbowDemandSupplier = elbowDemandSupplier;
    this.shouldCleanValues = shouldCleanValues;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double shoulderDemand = shoulderDemandSupplier.getAsDouble();
    double elbowDemand = elbowDemandSupplier.getAsDouble();

    if(shouldCleanValues){
      shoulderDemand = MathUtil.clamp(shoulderDemand, -1, 1);
      shoulderDemand = MathUtil.applyDeadband(shoulderDemand, Constants.Controllers.JOYSTICKS_DEADBAND_VALUE);
      elbowDemand = MathUtil.clamp(elbowDemand, -1.0, 1.0);
      elbowDemand = MathUtil.applyDeadband(elbowDemand, Constants.Controllers.JOYSTICKS_DEADBAND_VALUE);

      shoulderDemand *= ArmConstants.Controller.MULTIPLIER_SHOULDER;
      elbowDemand *= ArmConstants.Controller.MULTIPLIER_ELBOW;
    }

    shoulderDemand *= 12;
    elbowDemand *= 12;

    ArmValues<Double> feedforwardResults = arm.calculateFeedforward(
      arm.getShoulderAngle(),
      arm.getElbowAngle(),
      0,
      0,
      false
    );

    arm.setVoltageShoulder(shoulderDemand + feedforwardResults.shoulder);
    arm.setVoltageElbow(elbowDemand + feedforwardResults.elbow);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
