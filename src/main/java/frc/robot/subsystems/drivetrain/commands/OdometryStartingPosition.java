package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.startPos;

public class OdometryStartingPosition {
    
    public static Pose2d getNodePose(int posIndex, boolean reversed){
        Rotation2d rot = reversed ? Rotation2d.fromRotations(0) : Rotation2d.fromRotations(0.5);

        Pose2d pos;
        switch(posIndex){
            case 1:
                pos = new Pose2d(startPos.pos1x, startPos.pos1y, rot);
                break;
            
            case 2:
                pos = new Pose2d(startPos.pos2x, startPos.pos2y, rot);
                break;
                
            case 3:
                pos = new Pose2d(startPos.pos3x, startPos.pos3y, rot);
                break;
                
            case 4:
                pos = new Pose2d(startPos.pos4x, startPos.pos4y, rot);
                break;
                
            case 5:
                pos = new Pose2d(startPos.pos5x, startPos.pos5y, rot);
                break;
                
            case 6:
                pos = new Pose2d(startPos.pos6x, startPos.pos6y, rot);
                break;
                
            case 7:
                pos = new Pose2d(startPos.pos7x, startPos.pos7y, rot);
                break;
                
            case 8:
                pos = new Pose2d(startPos.pos8x, startPos.pos8y, rot);
                break;
                
            case 9:
                pos = new Pose2d(startPos.pos9x, startPos.pos9y, rot);
                break;

            default:
                pos = new Pose2d(startPos.defaultX, startPos.defaultY, rot);
                break;
        }
        if(!reversed){
            return pos;
        }else{
            Pose2d reversedPos = new Pose2d(pos.getX() + startPos.distanceFromSides, pos.getY(), rot);
            return reversedPos;
        }
    }
}
