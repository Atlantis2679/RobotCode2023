package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.ArmController;
import frc.robot.subsystems.arm.commands.ArmPositionsCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.BalanceOnChargeStation;
import frc.robot.subsystems.drivetrain.commands.DriveToDistance;
import frc.robot.subsystems.drivetrain.commands.GetOnChargeStation;
import frc.robot.subsystems.drivetrain.commands.TurnByDegree;
import frc.robot.subsystems.intake.Intake;

public final class Autos {
  public static Command releaseCone(Intake intake) {
    return new InstantCommand(() -> intake.setSpeed(-Constants.Autos.ReleaseCone.RELEASE_SPEED), intake)
        .andThen(new WaitCommand(Constants.Autos.ReleaseCone.RELEASE_TIME_SECONDS))
        .andThen(new InstantCommand(() -> intake.setSpeed(0), intake));
  }

  public static Command releaseConeSecond(Arm arm, Intake intake, Drivetrain drivetrain) {
    return driveForDistance(drivetrain, 0.20, 0.3, true)
        .andThen(
            ArmPositionsCommands.coneSecond(arm).withTimeout(Constants.Autos.releaseConeSecond.TIMEOUT_SECONDS_RAISE))
        .andThen(releaseCone(intake))
        .andThen(ArmPositionsCommands.rest(arm).withTimeout(Constants.Autos.releaseCubeSecond.TIMEOUT_SECONDS_LOWER));
  }

  public static Command releaseConeThird(Arm arm, Intake intake, Drivetrain drivetrain) {
    return new DriveToDistance(drivetrain, Constants.Autos.releaseConeThird.DRIVE_TO_DISTANCE_START)
        .andThen(
            ArmPositionsCommands.coneThird(arm).withTimeout(Constants.Autos.releaseConeThird.TIMEOUT_SECONDS_RAISE))
        .andThen(new DriveToDistance(drivetrain, Constants.Autos.releaseConeThird.DRIVE_TO_DISTANCE_BEFORE_RELEASE))
        .andThen(new WaitCommand(0.1))
        .andThen(releaseCone(intake))
        .andThen(new DriveToDistance(drivetrain, Constants.Autos.releaseConeThird.DRIVE_TO_DISTANCE_END))
        .andThen(ArmPositionsCommands.rest(arm).withTimeout(Constants.Autos.releaseConeThird.TIMEOUT_SECONDS_LOWER));
  }

  public static Command releaseCube(Intake intake) {
    return new InstantCommand(() -> intake.setSpeed(Constants.Autos.ReleaseCube.RELEASE_SPEED), intake)
        .andThen(new WaitCommand(Constants.Autos.ReleaseCube.RELEASE_TIME_SECONDS))
        .andThen(new InstantCommand(() -> intake.setSpeed(0), intake));
  }

  public static Command releaseCubeSecond(Arm arm, Intake intake, Drivetrain drivetrain) {
    return new DriveToDistance(drivetrain, Constants.Autos.releaseCubeSecond.DRIVE_TO_DISTANCE)
        .andThen(
            ArmPositionsCommands.cubeSecond(arm).withTimeout(Constants.Autos.releaseCubeSecond.TIMEOUT_SECONDS_RAISE))
        .andThen(releaseCube(intake))
        .andThen(ArmPositionsCommands.rest(arm).withTimeout(Constants.Autos.releaseCubeSecond.TIMEOUT_SECONDS_LOWER));
  }

  public static Command releaseCubeThird(Arm arm, Intake intake) {
    return ArmPositionsCommands.cubeThird(arm).withTimeout(Constants.Autos.releaseCubeThird.TIMEOUT_SECONDS_RAISE)
        .andThen(releaseCube(intake))
        .andThen(ArmPositionsCommands.rest(arm).withTimeout(Constants.Autos.releaseCubeThird.TIMEOUT_SECONDS_LOWER));
  }

  public static Command driveBackwardsOutsideCommunity(Drivetrain drivetrain, boolean turnOnFinish) {
    Command driveToDistanceCommand = new DriveToDistance(drivetrain,
        -Constants.Autos.DriveBackwardsOutsideCommunity.DISTANCE_METERS);

    return turnOnFinish
        ? driveToDistanceCommand.withTimeout(Constants.Autos.DriveBackwardsOutsideCommunity.TIMEOUT_SECONDS)
            .andThen(new TurnByDegree(drivetrain, Constants.Autos.DriveBackwardsOutsideCommunity.TURN_ENGLE))
        : driveToDistanceCommand.withTimeout(Constants.Autos.DriveBackwardsOutsideCommunity.TIMEOUT_SECONDS);
  }

  public static Command balanceChargeStation(Drivetrain drivetrain, Arm arm) {
    return new InstantCommand(() -> {
      arm.setSpeedShoulder(0);
      arm.setSpeedElbow(0);
    }, arm).andThen(
        new GetOnChargeStation(drivetrain).withTimeout(Constants.Autos.GetOnChargeStationAuto.TIMEOUT_SECONDS))
        .andThen(driveForDistance(
            drivetrain,
            Constants.Autos.BalanceOnChargeStationAuto.DISTANCE_TO_CLOSER_CENTER,
            Constants.Autos.BalanceOnChargeStationAuto.SPEED_TO_CLOSER_CENTER,
            Constants.Autos.BalanceOnChargeStationAuto.IS_REVERSED)
            .withTimeout(Constants.Autos.BalanceOnChargeStationAuto.GET_CLOSER_TO_CENTER_TIMEOUT))
        .andThen(new BalanceOnChargeStation(drivetrain));
    // return new InstantCommand(() -> {
    //   arm.setSpeedShoulder(0);
    //   arm.setSpeedElbow(0);
    // }, arm).andThen(
    //     new GetOnChargeStation(drivetrain).withTimeout(Constants.Autos.GetOnChargeStationAuto.TIMEOUT_SECONDS))
    //     .andThen(driveForDistance(
    //         drivetrain,
    //         Constants.Autos.BalanceOnChargeStationAuto.DISTANCE_TO_CLOSER_CENTER,
    //         Constants.Autos.BalanceOnChargeStationAuto.SPEED_TO_CLOSER_CENTER,
    //         Constants.Autos.BalanceOnChargeStationAuto.IS_REVERSED)
    //         .withTimeout(Constants.Autos.BalanceOnChargeStationAuto.GET_CLOSER_TO_CENTER_TIMEOUT))
    //     .andThen(() -> drivetrain.setSpeed(-0.2, -0.2), drivetrain)
    //     .andThen(new WaitUntilCommand(() -> drivetrain.getPitch() > -10))
    //     .andThen(() -> drivetrain.setSpeed(0, 0))
    //     .andThen(new WaitCommand(0.1))
    //     .andThen(new WaitCommand(0.5)).until(() -> Math.abs(drivetrain.getPitch()) < 7)
    //     .andThen(new BalanceOnChargeStation(drivetrain));
  }

  public static Command driveForDistance(Drivetrain drivetrain, double metersDistance, double speed,
      boolean isReversed) {
    final double speedDemand = speed * (isReversed ? -1 : 1);

    return new FunctionalCommand(() -> {
      drivetrain.resetEncoders();
      drivetrain.setSpeed(speedDemand, speedDemand);
    },
        () -> {
        },
        interrupted -> {
        },
        () -> {
          if (isReversed) {
            return drivetrain.getLeftDistanceMeters() < -metersDistance
                && drivetrain.getRightDistanceMeters() < -metersDistance;
          }

          return drivetrain.getLeftDistanceMeters() > metersDistance
              && drivetrain.getRightDistanceMeters() > metersDistance;
        },
        drivetrain);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
