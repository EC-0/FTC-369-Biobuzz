package org.firstinspires.ftc.teamcode.RoadRunner.SimpleAutos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class RedAutoDown extends LinearOpMode {

    MecanumDrive drivetrain;
    Pose2d initialPose = new Pose2d(-61,-40,90);

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new MecanumDrive(hardwareMap, initialPose);

        Action shootAndPark = drivetrain.actionBuilder(initialPose)
                // shoot
                .strafeToLinearHeading(new Vector2d(-45,-57), Math.toRadians(90))
                .build();

    }
}
