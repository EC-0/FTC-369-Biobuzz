package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class RobotLocalizer {
    MecanumDrive drive;
    public enum allianceColor {BLUE, RED}
    allianceColor alliance;

    public RobotLocalizer(HardwareMap hardwareMap, Pose2d startPose, allianceColor alliance){
        drive = new MecanumDrive(hardwareMap, startPose);
        this.alliance = alliance;
    }
    public Pose2d getBotPose(){
        return drive.localizer.getPose();
    }
    public void setBotPosition(Pose2d pose){
        drive.localizer.setPose(pose);
    }
}
