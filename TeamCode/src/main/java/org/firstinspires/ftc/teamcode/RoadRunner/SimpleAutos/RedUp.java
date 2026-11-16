package org.firstinspires.ftc.teamcode.RoadRunner.SimpleAutos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.BackgroundActions;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.PIDF_Controller;
import org.firstinspires.ftc.teamcode.Subsystems.RobotLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.Stopper;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class RedUp extends LinearOpMode {
    Limelight limelight;
    MecanumDrive drivetrain;
    Turret turret;
    Stopper stopper;
    Intake intake;
    RobotLocalizer localizer;
    BackgroundActions ba;
    PIDF_Controller pid;
    Pose2d initialPose = new Pose2d(62, -12, Math.toRadians(270));
    Pose2d garden = new Pose2d(62, -62, Math.toRadians(270));
    Pose2d shoot = new Pose2d(-62, -45, Math.toRadians(270));
    Pose2d park = new Pose2d(-20, -62, Math.toRadians(270));
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

        Action startToGarden = drivetrain.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(62, -62), Math.toRadians(270))
                .build();

        Action gardenToShoot = drivetrain.actionBuilder(garden)
                .strafeToLinearHeading(new Vector2d(62, -50), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-62, -45), Math.toRadians(270))
                .build();

        Action shootToGarden = drivetrain.actionBuilder(shoot)
                .strafeToLinearHeading(new Vector2d(62, -50), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-62, -62), Math.toRadians(270))
                .build();

        Action shootToPark = drivetrain.actionBuilder(shoot)
                .strafeToLinearHeading(new Vector2d(-20, -45), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-20, -62), Math.toRadians(270))
                .build();

        Action gardenToPark = drivetrain.actionBuilder(garden)
                .strafeToLinearHeading(new Vector2d(62, -40), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-20, -62), Math.toRadians(270))
                .build();

        RaceAction starttogarden = new RaceAction(
                startToGarden,
                ba.combinedActions()
        );

        RaceAction gardentoshoot = new RaceAction(
                gardenToShoot,
                ba.combinedActions()
        );

        RaceAction shoottogarden = new RaceAction(
                shootToGarden,
                ba.combinedActions()
        );

        RaceAction getReady = new RaceAction(
                new SleepAction(3.0),
                ba.update(),
                ba.setTurretPower()
        );

        waitForStart();

        stopper.setStopper(0.8);

        Actions.runBlocking(getReady);

        turret.regression(localizer.getDistanceToGoal());
        while (!(turret.getVelocity() >= (pid.calculate(turret.getVelocity())))) {}
        intake.intakePower(0.6);
        stopper.setStopper(0.0);

        Actions.runBlocking(new SleepAction(3.0));

        stopper.setStopper(0.8);

        Actions.runBlocking(starttogarden);

        Actions.runBlocking(new SleepAction(3.0));

        if (limelight.getTag() != null) {
            stopper.setStopper(0.0);
            Actions.runBlocking(new SleepAction(3.0));
            stopper.setStopper(0.8);
            intake.intakePower(0.0);
            Actions.runBlocking(gardenToPark);
        }
        else {
            Actions.runBlocking(gardentoshoot);
            if (limelight.getTag() != null) {
                stopper.setStopper(0.0);
                Actions.runBlocking(new SleepAction(3.0));
                intake.intakePower(0.0);
                stopper.setStopper(0.8);
                Actions.runBlocking(shootToPark);
            }
            else {
                Actions.runBlocking(shoottogarden);
                stopper.setStopper(0.0);
                Actions.runBlocking(new SleepAction(3.0));
                intake.intakePower(0.0);
                stopper.setStopper(0.8);
                Actions.runBlocking(gardenToPark);
            }
        }
    }
}
