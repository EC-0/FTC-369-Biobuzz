package org.firstinspires.ftc.teamcode.RoadRunner.SimpleAutos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.AdvancedAutos.Background;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.RobotLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.Stopper;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class RedAutoDown extends LinearOpMode {
    double tiptime = 5, shoottime = 4;

    MecanumDrive drivetrain;
    Background background;
    RobotLocalizer robotlocalizer;
    Intake intake;
    Turret turret;
    Limelight limelighter;
    Stopper stopper;
    Pose2d initialPose = new Pose2d(-60,-10,270);
    Pose2d shoot = new Pose2d(45,-35, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new MecanumDrive(hardwareMap, initialPose);
        limelighter = new Limelight(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);

        background = new Background(turret, intake);

        robotlocalizer = new RobotLocalizer(hardwareMap, initialPose, RobotLocalizer.allianceColor.RED);
        limelighter.switchPipeline(1);

        Action startToPark = drivetrain.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-50, -10), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-50,-60), Math.toRadians(270))
                .build();

        Action startToShoot = drivetrain.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-35, -35), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(45, -35), Math.toRadians(270))
                .build();

        Action shootToPark = drivetrain.actionBuilder(shoot)
                .strafeToLinearHeading(new Vector2d(-50, -35), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-50,-60), Math.toRadians(270))
                .build();

        Action shootToStart = drivetrain.actionBuilder(shoot)
                .strafeToLinearHeading(new Vector2d(-35, -35), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-60, -10), Math.toRadians(270))
                .build();

        waitForStart();

        new SleepAction(tiptime);

        intake.intakePower(0.5);
        stopper.setStopper(1.0);

        limelighter.getClosest();
        if (limelighter.getTag() != null) {
            stopper.setStopper(0.0);
            new SleepAction(shoottime);
            stopper.setStopper(1.0);
            Actions.runBlocking(Background.runWithTurret(startToPark, Background.runFlywheel(robotlocalizer.getDistanceToGoal()), Background.aim(robotlocalizer.angleToGoal())));
        }
        else {
            Actions.runBlocking(Background.runWithTurret(startToShoot, Background.runFlywheel(robotlocalizer.getDistanceToGoal()), Background.aim(robotlocalizer.angleToGoal())));
            limelighter.getClosest();
            if (limelighter.getTag() != null) {
                stopper.setStopper(0.0);
                new SleepAction(shoottime);
                stopper.setStopper(1.0);
                Actions.runBlocking(Background.runWithTurret(shootToPark, Background.runFlywheel(robotlocalizer.getDistanceToGoal()), Background.aim(robotlocalizer.angleToGoal())));
            }
            else {
                Actions.runBlocking(Background.runWithTurret(shootToStart, Background.runFlywheel(robotlocalizer.getDistanceToGoal()), Background.aim(robotlocalizer.angleToGoal())));
                stopper.setStopper(0.0);
                new SleepAction(shoottime);
                stopper.setStopper(1.0);
                Actions.runBlocking(Background.runWithTurret(startToPark, Background.runFlywheel(robotlocalizer.getDistanceToGoal()), Background.aim(robotlocalizer.angleToGoal())));
            }
        }

    }
}
