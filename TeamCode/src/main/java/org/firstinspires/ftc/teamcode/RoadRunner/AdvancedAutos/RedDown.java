package org.firstinspires.ftc.teamcode.RoadRunner.AdvancedAutos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous (name = "Red Down Autonomous")
public class RedDown extends LinearOpMode {
    boolean iseetags = true;
    double tiptime = 5.0;
    MecanumDrive drivetrain;
    Pose2d initialPose = new Pose2d(-60, -10, Math.toRadians(270));
    Pose2d park = new Pose2d(-50, -60, Math.toRadians(270));
    Pose2d shoot = new Pose2d(45,-35, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain = new MecanumDrive(hardwareMap, initialPose);

        Action startToPark = drivetrain.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-50, -10), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-50,-60), Math.toRadians(270))
                .build();

        Action startToShoot = drivetrain.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-35, -35), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(45, -35), Math.toRadians(270))
                .build();

        Action shootToStart = drivetrain.actionBuilder(shoot)
                .strafeToLinearHeading(new Vector2d(-35, -35), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-60, -10), Math.toRadians(270))
                .build();

        Action shootToPark = drivetrain.actionBuilder(shoot)
                .strafeToLinearHeading(new Vector2d(-50, -35), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-50,-60), Math.toRadians(270))
                .build();

        waitForStart();

        new SleepAction(tiptime);
        if (iseetags) {
            // shoot preloads
            // intake flower pollen
            // shoot preloads
            Actions.runBlocking(startToPark);
        }
        else {
            Actions.runBlocking(startToShoot);
            if (iseetags) {
                // shoots preloads
                Actions.runBlocking(shootToStart); // intakes flower pollen
                if (iseetags) {
                    // shoots flower pollen
                    Actions.runBlocking(startToPark);
                }
                else {
                    Actions.runBlocking(startToShoot);
                    if (iseetags) {
                        // shoots flower pollen
                        Actions.runBlocking(shootToPark);
                    }
                    else {
                        Actions.runBlocking(shootToStart);
                        // shoot flower pollen
                        Actions.runBlocking(startToPark);
                    }
                }
            }
            else {
                Actions.runBlocking(shootToStart);
                // shoots preloads
                // intakes flower pollen
                // shoots flower pollen
                Actions.runBlocking(startToPark);
            }
        }
    }
}
