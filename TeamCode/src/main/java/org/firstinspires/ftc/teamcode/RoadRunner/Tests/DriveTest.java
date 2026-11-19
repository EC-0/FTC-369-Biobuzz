package org.firstinspires.ftc.teamcode.RoadRunner.Tests;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous (name = "Drive Test")
public class DriveTest extends LinearOpMode {
    MecanumDrive drivetrain;
    Pose2d initialPose = new Pose2d(-62,-62,Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new MecanumDrive(hardwareMap,initialPose);

        Action test = drivetrain.actionBuilder(initialPose)

                .strafeToLinearHeading(new Vector2d(-62, -32), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-32, -32), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-32, -62), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-62, -62), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-32, -32), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-62, -32), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-32, -62), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-62, -62), Math.toRadians(90))
                .build();

        waitForStart();

        Actions.runBlocking(test);

    }
}
