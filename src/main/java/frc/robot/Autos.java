package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.ArmController;
import frc.robot.subsystems.arm.commands.ArmPositionsCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.AutoPath;
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
      arm.setVoltageShoulder(0);
      arm.setVoltageElbow(0);
    }, arm).andThen(
        new GetOnChargeStation(drivetrain).withTimeout(Constants.Autos.GetOnChargeStationAuto.TIMEOUT_SECONDS))
        .andThen(driveForDistance(
            drivetrain,
            Constants.Autos.BalanceOnChargeStationAuto.DISTANCE_TO_CLOSER_CENTER,
            Constants.Autos.BalanceOnChargeStationAuto.SPEED_TO_CLOSER_CENTER,
            Constants.Autos.BalanceOnChargeStationAuto.IS_REVERSED)
            .withTimeout(Constants.Autos.BalanceOnChargeStationAuto.GET_CLOSER_TO_CENTER_TIMEOUT))

        // .andThen(() -> drivetrain.setSpeed(-0.2, -0.2), drivetrain)
        // .andThen(new WaitCommand(0.2))
        // .andThen(new WaitUntilCommand(() -> drivetrain.getPitch() > -5))
        // .andThen(() -> drivetrain.setSpeed(0, 0))
        // .andThen(new WaitCommand(0.2))
        // .andThen(new WaitCommand(0.5)).until(() -> Math.abs(drivetrain.getPitch()) >
        // 10)

        .andThen(new BalanceOnChargeStation(drivetrain));
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

  public static Command driveAutoPath(Drivetrain drivetrain) {
    final DifferentialDriveKinematics Kinematics = new DifferentialDriveKinematics(AutoPath.WHEEL_DISTANCE);

    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            AutoPath.KS,
            AutoPath.KV,
            AutoPath.KA),
        Kinematics,
        10);

    TrajectoryConfig config = new TrajectoryConfig(
        AutoPath.MAX_VELOCITY,
        AutoPath.MAX_ACCELERATION)
        .setKinematics(Kinematics)
        .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),

        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),

        new Pose2d(3, 0, new Rotation2d(0)),

        config);

    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
    drivetrain.setTrajectory(exampleTrajectory);

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        drivetrain::getPose,
        new RamseteController(AutoPath.K_RAMESTE_B, AutoPath.K_RAMESTE_ZETA),
        new SimpleMotorFeedforward(
            AutoPath.KS,
            AutoPath.KV,
            AutoPath.KA),
        Kinematics,
        drivetrain::getWheelSpeeds,
        new PIDController(AutoPath.KP, 0, 0),
        new PIDController(AutoPath.KP, 0, 0),
        drivetrain::setVoltage,
        drivetrain);

    return ramseteCommand.andThen(() -> drivetrain.setVoltage(0, 0));
  }

  public static Command drivePathPlanner(Drivetrain drivetrain) {
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", new PathConstraints(4, 3));
    return new RamseteCommand(examplePath,
        drivetrain::getPose,
        new RamseteController(3, 7),
        new SimpleMotorFeedforward(
            AutoPath.KS,
            AutoPath.KV,
            AutoPath.KA),
        new DifferentialDriveKinematics(AutoPath.WHEEL_DISTANCE),
        drivetrain::getWheelSpeeds,
        new PIDController(0, 0, 0),
        new PIDController(0, 0, 0),
        drivetrain::setVoltage,
        drivetrain);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
