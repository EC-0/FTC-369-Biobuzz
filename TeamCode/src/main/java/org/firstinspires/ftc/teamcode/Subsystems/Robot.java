package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    Drivetrain drive;
    RobotLocalizer localizer;
    Limelight limelight;
    Intake intake;
    Shooter outtake;
    Turret turret;
    Stopper stopper;
    RobotLocalizer.allianceColor allianceColor;
    Telemetry telemetry;

    public Robot(Telemetry telemetry, HardwareMap hardwareMap, RobotLocalizer.allianceColor allianceColor, Pose2d startPose){
        drive = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);
        this.allianceColor = allianceColor;
        localizer = new RobotLocalizer(hardwareMap, startPose, allianceColor);

        limelight.switchPipeline(allianceColor == RobotLocalizer.allianceColor.BLUE ? 0: 1);
        this.telemetry = telemetry;
    }
    public void start(){

    }
}
