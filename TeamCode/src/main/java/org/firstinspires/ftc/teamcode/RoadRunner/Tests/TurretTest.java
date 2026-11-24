package org.firstinspires.ftc.teamcode.RoadRunner.Tests;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.RobotLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Stopper;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.RoadRunner.BackgroundActions;

@Autonomous (name = "Turret Test")
public class TurretTest extends LinearOpMode {

    Turret turret ;
    MecanumDrive drivetrain;
    Intake intake;
    Stopper stopper;
    RobotLocalizer localizer;
    Limelight limelight;
    BackgroundActions ba;
    Shooter shooter;
    Pose2d initialPose = new Pose2d(-62,-62,90);

    @Override
    public void runOpMode() throws InterruptedException {
        turret = new Turret(hardwareMap);
        drivetrain = new MecanumDrive(hardwareMap, initialPose);
        intake = new Intake(hardwareMap);
        localizer = new RobotLocalizer(hardwareMap, initialPose, RobotLocalizer.allianceColor.RED);
        stopper = new Stopper(hardwareMap);
        limelight = new Limelight(hardwareMap);
        ba = new BackgroundActions(turret, localizer, limelight, shooter, stopper);

        Action path1 = drivetrain.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(55, 0), Math.toRadians(90))
                .build();

        Action path2 = drivetrain.actionBuilder(new Pose2d(55,0,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(50, -40), Math.toRadians(180))
                .build();

        Action path3 = drivetrain.actionBuilder(new Pose2d(50,-40, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(45, -10), Math.toRadians(270))
                .build();

        Action path4 = drivetrain.actionBuilder(new Pose2d(45, -10, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(50, 30), Math.toRadians(0))
                .build();

        Action path5 = drivetrain.actionBuilder(new Pose2d(50, 30, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(55, -20), Math.toRadians(90))
                .build();

        SequentialAction drive = new SequentialAction(
                path1,
                path2,
                path3,
                path4,
                path5
        );

        RaceAction test = new RaceAction(
                ba.combinedActions(1650),
                ba.updateLocalizer(),
                drive
        );

        waitForStart();

        stopper.setStopper(0.8);
        Actions.runBlocking(test);
        shooter.regression(localizer.getDistanceToGoal());
        turret.update(localizer.angleToGoal());
        intake.intakePower(0.6);
        stopper.setStopper(0.0);

        Actions.runBlocking(new SleepAction(5.0));

        stopper.setStopper(0.8);
    }
}
