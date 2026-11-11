package org.firstinspires.ftc.teamcode.RoadRunner.AdvancedAutos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous (name = "Red Up Autonomous")
public class RedUp extends LinearOpMode {

    // iseetags is just a placeholder for actual ll detection right now
    boolean iseetags = true;
    double tiptime = 5.0;

    Pose2d initialPose = new Pose2d(62,-12,Math.toRadians(270));
    Pose2d garden = new Pose2d(62,-57, Math.toRadians(270));
    Pose2d shoot = new Pose2d(-55,-45, Math.toRadians(270));
    Pose2d flower = new Pose2d(24,-56, Math.toRadians(270));
    Pose2d park = new Pose2d(-20,-55, Math.toRadians(270));
    MecanumDrive drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new MecanumDrive(hardwareMap, initialPose);

        Action startToGarden = drivetrain.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(62,-57), Math.toRadians(270))
                .build();

        Action gardenToShoot = drivetrain.actionBuilder(garden)
                .strafeToLinearHeading(new Vector2d(-55,-45), Math.toRadians(270))
                .build();

        Action shootToPark = drivetrain.actionBuilder(shoot)
                .strafeToLinearHeading(new Vector2d(-20,-45), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-20,-55), Math.toRadians(270))
                .build();

        Action gardenToFlower = drivetrain.actionBuilder(garden)
                .strafeToLinearHeading(new Vector2d(62,-50), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(24, -56), Math.toRadians(270))
                .build();

        Action flowerToGarden = drivetrain.actionBuilder(flower)
                .strafeToLinearHeading(new Vector2d(62,-50), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(62, -57), Math.toRadians(270))
                .build();

        Action gardenToPark = drivetrain.actionBuilder(garden)
                .strafeToLinearHeading(new Vector2d(-25,-40), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-20,-55), Math.toRadians(270))
                .build();

        Action shootToFlower = drivetrain.actionBuilder(shoot)
                .strafeToLinearHeading(new Vector2d(24,-56), Math.toRadians(270))
                .build();

        Action flowerToShoot = drivetrain.actionBuilder(flower)
                .strafeToLinearHeading(new Vector2d(-55,-45), Math.toRadians(270))
                .build();

        Action shootToGarden = drivetrain.actionBuilder(shoot)
                .strafeToLinearHeading(new Vector2d(62, -57), Math.toRadians(270))
                .build();

        waitForStart();

        // shoots preloads
        new SleepAction(tiptime); // waits for potential tip
        if (iseetags) {
            Actions.runBlocking(startToGarden); // intakes garden pollen
            // shoots garden pollen
            new SleepAction(tiptime); // waits for potential tip
            if (iseetags) {
                Actions.runBlocking(gardenToFlower); // intakes flower pollen
                Actions.runBlocking(flowerToGarden);
                // shoots flower pollen
                Actions.runBlocking(gardenToPark);
            }
            else {
                Actions.runBlocking(gardenToFlower); // intakes flower pollen
                Actions.runBlocking(flowerToShoot);
                if (iseetags) {
                    // shoots flower pollen
                    Actions.runBlocking(shootToPark);
                }
                else {
                    Actions.runBlocking(shootToGarden);
                    // shoots flower pollen
                    Actions.runBlocking(gardenToPark);
                }
            }
        }
        else {
            Actions.runBlocking(startToGarden); // intakes garden pollen
            Actions.runBlocking(gardenToShoot);
            if (iseetags) {
                // shoots garden pollen
                new SleepAction(tiptime); // waits for potential tips
                Actions.runBlocking(shootToFlower); // intakes flower pollen
                Actions.runBlocking(flowerToShoot);
                if (iseetags) {
                    // shoots flower pollen
                    Actions.runBlocking(shootToPark);
                }
                else {
                    Actions.runBlocking(shootToGarden);
                    // shoots flower pollen
                    Actions.runBlocking(gardenToPark);
                }
            }
            else {
                Actions.runBlocking(shootToGarden);
                // shoots pollen from garden
                Actions.runBlocking(gardenToFlower); // intakes flower pollen
                Actions.runBlocking(flowerToGarden);
                // shoots pollen from flower
                Actions.runBlocking(gardenToPark);
            }
        }
    }
}
