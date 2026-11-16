package org.firstinspires.ftc.teamcode.RoadRunner.SimpleAutos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.BackgroundActions;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.PIDF_Controller;
import org.firstinspires.ftc.teamcode.Subsystems.RobotLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.Stopper;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class RedDown extends LinearOpMode {

    Limelight limelight;
    MecanumDrive drivetrain;
    Turret turret;
    Stopper stopper;
    Intake intake;
    RobotLocalizer localizer;
    BackgroundActions ba;
    PIDF_Controller pid;
    Pose2d initialPose = new Pose2d(-62,-10, Math.toRadians(270));
    Pose2d shoot = new Pose2d(50, -50, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = new Limelight(hardwareMap);
        drivetrain = new MecanumDrive(hardwareMap, initialPose);
        turret = new Turret(hardwareMap);
        stopper = new Stopper(hardwareMap);
        intake = new Intake(hardwareMap);
        localizer = new RobotLocalizer(hardwareMap ,initialPose, RobotLocalizer.allianceColor.RED);
        ba = new BackgroundActions(turret, localizer, limelight);
        pid = new PIDF_Controller(13.5, 0.05 , 0.01, 20,-.009625);

        Action startToPark = drivetrain.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-50,-10), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-50,-62), Math.toRadians(270))
                .build();

        Action startToShoot = drivetrain.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-50,-10), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-50,-50), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(50,-50), Math.toRadians(270))
                .build();

        RaceAction getReady = new RaceAction(
                new SleepAction(5.5),
                ba.combinedActions()
        );

        waitForStart();

        stopper.setStopper(0.8);

        Actions.runBlocking(getReady);

        Actions.runBlocking(ba.getClosest());

    }
}
