package frc.robot.subsystems.intake.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

public class IntakeController extends CommandBase {
  private final Intake intake;
  private final DoubleSupplier forwardDemandSupplier;
  private final DoubleSupplier backwardDemandSupplier;

  public IntakeController(Intake intake, DoubleSupplier forwardDemandSupplier, DoubleSupplier backwardDemandSupplier) {
    this.intake = intake;
    addRequirements(intake);
    this.forwardDemandSupplier = forwardDemandSupplier;
    this.backwardDemandSupplier = backwardDemandSupplier;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {    
    double forward = forwardDemandSupplier.getAsDouble();
    double backward = backwardDemandSupplier.getAsDouble();
    double speed = backward > forward ? -backward : forward;

    intake.setSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
