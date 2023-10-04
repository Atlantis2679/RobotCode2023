package frc.robot;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.ArmController;
import frc.robot.subsystems.arm.commands.ArmPositionsCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.ArcadeDrive;
import frc.robot.subsystems.drivetrain.commands.OdometryStartingPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeController;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    private final Arm arm = new Arm();
    public final CommandXboxController driverController = new CommandXboxController(Constants.Controllers.DRIVER_PORT);
    public final CommandXboxController operatorController = new CommandXboxController(
            Constants.Controllers.OPERATOR_PORT);

    private interface CommandSupplier {
        Command getCommand();
    }

    private final LoggedDashboardChooser<CommandSupplier> firstAutoCommandChooser = new LoggedDashboardChooser<>(
            "First Auto Command");
    private final LoggedDashboardChooser<CommandSupplier> secondAutoCommandChooser = new LoggedDashboardChooser<>(
            "Second Auto Command");

    private final LoggedDashboardChooser<Integer> startingPositionChooser = new LoggedDashboardChooser<>(
        "Start Position Chooser");

    private final LoggedDashboardChooser<Alliance> teamAlliance = new LoggedDashboardChooser<>(
        "Team Alliance (Color)");

//     private final LoggedDashboardChooser<> teamColor = new LoggedDashboardChooser<>(
//         "current match team color");

    public RobotContainer() {
        if (Robot.isReal()) {
            UsbCamera camera = CameraServer.startAutomaticCapture();
            camera.setVideoMode(
                    VideoMode.PixelFormat.kMJPEG,
                    Constants.Camera.WIDTH,
                    Constants.Camera.HEIGHT,
                    Constants.Camera.FPS);
        }

        configureBindings();

        // first auto command
        firstAutoCommandChooser.addDefaultOption(
                "Release Cone",
                () -> Autos.releaseCone(intake));

        // firstAutoCommandChooser.setDefaultOption(
        // "Release Cone Second",
        // () -> Autos.releaseConeSecond(arm, intake, drivetrain));

        // firstAutoCommandChooser.setDefaultOption(
        // "Release Cone Third",
        // () -> Autos.releaseConeThird(arm, intake, drivetrain));

        firstAutoCommandChooser.addOption(
                "Release Cube",
                () -> Autos.releaseCube(intake));

        firstAutoCommandChooser.addOption(
                "Release Cube Second",
                () -> Autos.releaseCubeSecond(arm, intake, drivetrain));

        firstAutoCommandChooser.addOption(
                "Release Cube Third",
                () -> Autos.releaseCubeThird(arm, intake));

        firstAutoCommandChooser.addOption(
                "None",
                () -> new InstantCommand());

                
        // second auto command
        secondAutoCommandChooser.addDefaultOption(
                "Balance On Charge Station",
                () -> Autos.balanceChargeStation(
                        drivetrain,
                        arm,
                        true));

        secondAutoCommandChooser.addOption(
                "Drive Backwards Outside Community and turn 180 degrease",
                () -> Autos.driveBackwardsOutsideCommunity(drivetrain, true));

        secondAutoCommandChooser.addOption(
                "Drive Backwards Outside Community",
                () -> Autos.driveBackwardsOutsideCommunity(drivetrain, false));

        secondAutoCommandChooser.addOption(
                "None",
                () -> new InstantCommand());

        startingPositionChooser.addDefaultOption("default", 0);

        startingPositionChooser.addOption("1",1);

        startingPositionChooser.addOption("2",2);

        startingPositionChooser.addOption("3",3);

        startingPositionChooser.addOption("4",4);

        startingPositionChooser.addOption("5",5);

        startingPositionChooser.addOption("6",6);

        startingPositionChooser.addOption("7",7);

        startingPositionChooser.addOption("8",8);

        startingPositionChooser.addOption("9",9);

        teamAlliance.addDefaultOption("Blue", Alliance.Blue);

        teamAlliance.addOption("Red", Alliance.Red);

        // teamColor.addOption("red team", null);

        // teamColor.addOption("blue team", null);
    }

    private void configureBindings() {
        // driver

        drivetrain.setDefaultCommand(new ArcadeDrive(
                drivetrain,
                () -> -driverController.getLeftY(),
                driverController::getRightX,
                () -> driverController.rightBumper().getAsBoolean(),
                () -> driverController.rightBumper().getAsBoolean()));

        // operator

        intake.setDefaultCommand(new IntakeController(
                intake,
                operatorController::getRightTriggerAxis,
                operatorController::getLeftTriggerAxis));

        arm.setDefaultCommand(new ArmController(arm,
                () -> -operatorController.getLeftY(),
                () -> -operatorController.getRightY(),
                true));

        operatorController.a().onTrue(ArmPositionsCommands.rest(arm));
        operatorController.x().onTrue(ArmPositionsCommands.cubeSecond(arm));
        operatorController.y().onTrue(ArmPositionsCommands.cubeThird(arm));
        operatorController.b().onTrue(ArmPositionsCommands.feeder(arm));
        operatorController.povLeft().onTrue(ArmPositionsCommands.coneSecond(arm));
        operatorController.povUp().onTrue(ArmPositionsCommands.coneThird(arm));
        operatorController.povDown().onTrue(ArmPositionsCommands.floor(arm));
        operatorController.povRight().onTrue(ArmPositionsCommands.floorTouchAndGo(arm));

        operatorController.leftBumper().onTrue(new InstantCommand(
                () -> arm.setEmergencyMode(!arm.getEmergencyMode())));
        operatorController.rightBumper().whileTrue(
                new RunCommand(() -> {
                    arm.setVoltageElbow(0);
                    arm.setVoltageShoulder(0);
                }, arm));

        // operatorController.rightStick().onTrue(new InstantCommand(() ->
        // arm.resetEncoders()));
    }

    private void beforeAuto(){
        if(teamAlliance.get() == Alliance.Blue){
                drivetrain.resetOdometry(OdometryStartingPosition.getNodePose(startingPositionChooser.get(), false));
        }else if(teamAlliance.get() == Alliance.Red){
                int index = 10 - startingPositionChooser.get();
                drivetrain.resetOdometry(OdometryStartingPosition.getNodePose(index, true));
        }

        drivetrain.resetPitch();
    }

    public Command getAutonomousCommand() {
        return new InstantCommand(this::beforeAuto)
                .andThen(firstAutoCommandChooser.get().getCommand())
                .andThen(secondAutoCommandChooser.get() .getCommand());
    }
}
