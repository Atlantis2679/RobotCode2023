package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeController extends CommandBase {
  private Intake intake;
  private DoubleSupplier forwardDemandSupplier;
  private DoubleSupplier backwardDemandSupplier;

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
