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
import frc.robot.subsystems.drivetrain.commands.driveForDistance;
import frc.robot.subsystems.intake.Intake;

import static frc.robot.Constants.Autos.*;

public final class Autos {
        public static Command releaseCone(Intake intake) {
                return new InstantCommand(() -> intake.setSpeed(-ReleaseConeConstants.RELEASE_SPEED), intake)
                                .andThen(new WaitCommand(ReleaseConeConstants.RELEASE_TIME_SECONDS))
                                .andThen(new InstantCommand(() -> intake.setSpeed(0), intake));
        }

        public static Command releaseConeSecond(Arm arm, Intake intake, Drivetrain drivetrain) {
                return driveForDistance(drivetrain, 0.20, 0.3, true)
                                .andThen(
                                                ArmPositionsCommands.coneSecond(arm)
                                                                .withTimeout(releaseConeSecondConstants.TIMEOUT_SECONDS_RAISE))
                                .andThen(releaseCone(intake))
                                .andThen(ArmPositionsCommands.rest(arm)
                                                .withTimeout(releaseCubeSecondConstants.TIMEOUT_SECONDS_LOWER));
        }

        public static Command releaseConeThird(Arm arm, Intake intake, Drivetrain drivetrain) {
                return new DriveToDistance(drivetrain, releaseConeThirdConstants.DRIVE_TO_DISTANCE_START)
                                .andThen(
                                                ArmPositionsCommands.coneThird(arm)
                                                                .withTimeout(releaseConeThirdConstants.TIMEOUT_SECONDS_RAISE))
                                .andThen(new DriveToDistance(drivetrain,
                                                releaseConeThirdConstants.DRIVE_TO_DISTANCE_BEFORE_RELEASE))
                                .andThen(new WaitCommand(0.1))
                                .andThen(releaseCone(intake))
                                .andThen(new DriveToDistance(drivetrain,
                                                releaseConeThirdConstants.DRIVE_TO_DISTANCE_END))
                                .andThen(ArmPositionsCommands.rest(arm)
                                                .withTimeout(releaseConeThirdConstants.TIMEOUT_SECONDS_LOWER));
        }

        public static Command releaseCube(Intake intake) {
                return new InstantCommand(() -> intake.setSpeed(ReleaseCubeConstants.RELEASE_SPEED), intake)
                                .andThen(new WaitCommand(ReleaseCubeConstants.RELEASE_TIME_SECONDS))
                                .andThen(new InstantCommand(() -> intake.setSpeed(0), intake));
        }

        public static Command releaseCubeSecond(Arm arm, Intake intake, Drivetrain drivetrain) {
                return new DriveToDistance(drivetrain, releaseCubeSecondConstants.DRIVE_TO_DISTANCE)
                                .andThen(
                                                ArmPositionsCommands.cubeSecond(arm)
                                                                .withTimeout(releaseCubeSecondConstants.TIMEOUT_SECONDS_RAISE))
                                .andThen(releaseCube(intake))
                                .andThen(ArmPositionsCommands.rest(arm)
                                                .withTimeout(releaseCubeSecondConstants.TIMEOUT_SECONDS_LOWER));
        }

        public static Command releaseCubeThird(Arm arm, Intake intake) {
                return ArmPositionsCommands.cubeThirdMoveIndividualyFromRest(arm)
                                .withTimeout(releaseCubeThirdConstants.TIMEOUT_SECONDS_RAISE)
                                .andThen(releaseCube(intake))
                                .andThen(ArmPositionsCommands.rest(arm)
                                                .withTimeout(releaseCubeThirdConstants.TIMEOUT_SECONDS_LOWER));
        }

        public static Command driveBackwardsOutsideCommunity(Drivetrain drivetrain, boolean turnOnFinish) {
                Command driveToDistanceCommand = new DriveToDistance(drivetrain,
                                -DriveBackwardsOutsideCommunityConstants.DISTANCE_METERS);

                return turnOnFinish
                                ? driveToDistanceCommand
                                                .withTimeout(DriveBackwardsOutsideCommunityConstants.TIMEOUT_SECONDS)
                                                .andThen(
                                                                new TurnByDegree(drivetrain,
                                                                                DriveBackwardsOutsideCommunityConstants.TURN_ENGLE))
                                : driveToDistanceCommand
                                                .withTimeout(DriveBackwardsOutsideCommunityConstants.TIMEOUT_SECONDS);
        }

        public static Command balanceChargeStation(Drivetrain drivetrain, Arm arm, boolean isReversed) {
                return new InstantCommand(() -> {
                        arm.setVoltageShoulder(0);
                        arm.setVoltageElbow(0);
                }, arm).andThen(
                                new GetOnChargeStation(drivetrain, isReversed)
                                                .withTimeout(GetOnChargeStationConstants.TIMEOUT_SECONDS))
                                .andThen(() -> drivetrain.setSpeed(
                                                -BalanceOnChargeStationConstants.SPEED_TO_CLOSER_CENTER,
                                                -BalanceOnChargeStationConstants.SPEED_TO_CLOSER_CENTER), drivetrain)
                                .andThen(new WaitUntilCommand(() -> drivetrain.getPitch() > -18).withTimeout(0.15))
                                .andThen(driveForDistance(
                                                drivetrain,
                                                BalanceOnChargeStationConstants.DISTANCE_TO_CLOSER_CENTER,
                                                BalanceOnChargeStationConstants.SPEED_TO_CLOSER_CENTER,
                                                isReversed)
                                                .withTimeout(BalanceOnChargeStationConstants.GET_CLOSER_TO_CENTER_TIMEOUT))
                                .andThen(new BalanceOnChargeStation(drivetrain));
        }

    public static Command driveForDistance(
            Drivetrain drivetrain,
            double metersDistance,
            double speed,
            boolean isReversed) {
        return new driveForDistance(drivetrain);
        }

        // return new FunctionalCommand(() -> {
        // drivetrain.resetEncoders();
        // drivetrain.setSpeed(speedDemand, speedDemand);
        // },
        // () -> {
        // },
        // interrupted -> {
        // },
        // () -> {
        // if (isReversed) {
        // return drivetrain.getLeftDistanceMeters() < -metersDistance
        // && drivetrain.getRightDistanceMeters() < -metersDistance;
        // }

        // return drivetrain.getLeftDistanceMeters() > metersDistance
        // && drivetrain.getRightDistanceMeters() > metersDistance;
        // },
        // drivetrain);
        // }

        private Autos() {
                throw new UnsupportedOperationException("This is a utility class!");
        }
}
