package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class RobotLocalizer {
    MecanumDrive drive;
    public enum allianceColor {BLUE, RED}
    allianceColor alliance;
    public double GOAL_X;
    public double GOAL_Y = 10;

    public RobotLocalizer(HardwareMap hardwareMap, Pose2d startPose, allianceColor alliance){
        drive = new MecanumDrive(hardwareMap, startPose);
        this.alliance = alliance;

        if(alliance == allianceColor.BLUE){//FIND THE ACTUAL VALUES FOR THESE
            GOAL_X = 10;
        }
        else{
            GOAL_X = -10;
        }
    }
    public void updateBotPosition(){
        drive.updatePoseEstimate();
    }
    public Pose2d getBotPose(){
        return drive.localizer.getPose();
    }
    public double getDistanceToGoal(){
        Pose2d currentPose = getBotPose();
        goalTarget();
        return Math.hypot((Math.abs(GOAL_X - currentPose.position.x)), (Math.abs(GOAL_Y - currentPose.position.y)));
    }
    public double angleToGoal(){
        Pose2d currentPose = getBotPose();
        goalTarget();

        double targetFieldAngle = Math.atan2(GOAL_X - currentPose.position.x, GOAL_Y - currentPose.position.y);
        double targetRelativeAngle = AngleUnit.normalizeRadians(targetFieldAngle - currentPose.heading.real);
        return Math.toDegrees(targetRelativeAngle) % 360;
    }
    public void goalTarget(){
        Pose2d currentPose = getBotPose();
        if(currentPose.position.y <0){
            GOAL_Y = -10;
        }
        else{
            GOAL_Y = 10;
        }
    }
    public void setBotPosition(Pose2d pose){
        drive.localizer.setPose(pose);
    }
}
