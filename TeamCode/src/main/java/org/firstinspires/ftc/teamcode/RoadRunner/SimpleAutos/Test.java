package org.firstinspires.ftc.teamcode.RoadRunner.SimpleAutos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Test")
public class Test extends LinearOpMode {

    MecanumDrive drivetrain;
    Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new MecanumDrive(hardwareMap, initialPose);

        Action line1 = drivetrain.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(0, 30), Math.toRadians(-45))
                .strafeToLinearHeading(new Vector2d(30,0), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(0,0), Math.toRadians(90))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(line1);
    }
}
