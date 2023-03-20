package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;

public class ArmPositionsCommands {
    public static Command rest(Arm arm) {
        return new MoveArmToPosition(
                arm,
                ArmConstants.ANGLE_REST_SHOULDER,
                ArmConstants.ANGLE_REST_ELBOW);
    }

    public static Command cubeThird(Arm arm) {
        return new MoveArmToPosition(
                arm,
                ArmConstants.ANGLE_THIRD_CUBE_SHOULDER,
                ArmConstants.ANGLE_THIRD_CUBE_ELBOW);
    }

    public static Command cubeSecond(Arm arm) {
        return new MoveArmToPosition(
                arm,
                ArmConstants.ANGLE_SECOND_CUBE_SHOULDER,
                ArmConstants.ANGLE_SECOND_CUBE_ELBOW);
    }

    public static Command coneThird(Arm arm) {
        return new MoveArmToPosition(
                arm,
                ArmConstants.ANGLE_THIRD_CONE_SHOULDER,
                ArmConstants.ANGLE_THIRD_CONE_ELBOW);
    }

    public static Command coneSecond(Arm arm) {
        return new MoveArmToPosition(
                arm,
                ArmConstants.ANGLE_SECOND_CONE_SHOULDER,
                ArmConstants.ANGLE_SECOND_CONE_ELBOW);
    }

    public static Command feeder(Arm arm) {
        return new MoveArmToPosition(
                arm,
                ArmConstants.ANGLE_FEEDER_SHOULDER,
                ArmConstants.ANGLE_FEEDER_ELBOW);
    }

    public static Command floor(Arm arm) {
        return new MoveArmToPosition(
                arm,
                ArmConstants.ANGLE_FLOOR_SHOULDER,
                ArmConstants.ANGLE_FLOOR_ELBOW);
    }

    public static Command floorTouchAndGo(Arm arm) {
        return new MoveArmToPosition(
                arm,
                ArmConstants.ANGLE_TOUCH_AND_GO_SHOULDER,
                ArmConstants.ANGLE_TOUCH_AND_GO_ELBOW);
    }
}
