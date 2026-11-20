package org.firstinspires.ftc.teamcode.RoadRunner.SimpleAutos;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.BackgroundActions;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.PIDF_Controller;
import org.firstinspires.ftc.teamcode.Subsystems.RobotLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Stopper;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Autonomous (name = "RedUp")
public class RedUp extends LinearOpMode {
    double shootTime = 6, tipTime = 5, lenience = 50, targetVelocity;
    Limelight limelight;
    MecanumDrive drivetrain;
    Turret turret;
    Shooter shooter;
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
        shooter = new Shooter(hardwareMap);
        stopper = new Stopper(hardwareMap);
        intake = new Intake(hardwareMap);
        localizer = new RobotLocalizer(hardwareMap ,initialPose, RobotLocalizer.allianceColor.RED);
        ba = new BackgroundActions(turret, localizer, limelight, shooter);
        pid = new PIDF_Controller(13.5, 0.05 , 0.01, 20,-.009625);

        limelight.switchPipeline(1);

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

        Action gardenToStart = drivetrain.actionBuilder(garden)
                .strafeToLinearHeading(new Vector2d(62,-12), Math.toRadians(270))
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

        RaceAction waitForTip = new RaceAction(
                new SleepAction(tipTime),
                ba.getClosest(),
                ba.dontGetTag()
        );

        RaceAction refresh = new RaceAction(
                new SleepAction(1.0),
                ba.combinedActions()
        );

        RaceAction refreshAndWait = new RaceAction(
                new SleepAction(shootTime),
                ba.combinedActions()
        );

        RaceAction gardentostart = new RaceAction(
                gardenToStart,
                ba.combinedActions()
        );

        waitForStart();

        // turns on the stopper at the start
        stopper.setStopper(0.8);

        // has 3 seconds of setting up, by running the motor and aiming the turret
        Actions.runBlocking(getReady);

        // starts fine adjusting the flywheel power
        shooter.regression(localizer.getDistanceToGoal());

        // does not continue until the flywheel has reached in a certain threshold of power
        targetVelocity =(3.36048 * localizer.getDistanceToGoal()) + 1325.87065;
        while (opModeIsActive() && !(shooter.getShooterVelocity() > targetVelocity - lenience)) {
            targetVelocity =(3.36048 * localizer.getDistanceToGoal()) + 1325.87065;
            shooter.regression(localizer.getDistanceToGoal());
        }

        // begins shooting the balls out by not stopping and turning on the intake
        intake.intakePower(0.9);
        stopper.setStopper(0.0);

        // stops for a certain amount of time to let all the balls shoot out
        Actions.runBlocking(refreshAndWait);

        // turns on the stopper again
        stopper.setStopper(0.8);

        // goes to intake more balls from the garden
        Actions.runBlocking(starttogarden);

        // goes back to start
        Actions.runBlocking(gardentostart);

        // either waits 6 seconds or until it stops detecting april tags
        Actions.runBlocking(waitForTip);

        // just updates the limelight again
        limelight.getClosest();

        // detects if the hive is still up
        if (limelight.getTag() != null) {

            // just refreshes the aim and flywheel power
            Actions.runBlocking(refresh);

            // waits until flywheel power is good enough
            targetVelocity =(3.36048 * localizer.getDistanceToGoal()) + 1325.87065;
            while (opModeIsActive() && !((shooter.getShooterVelocity() > targetVelocity - lenience ) && (shooter.getShooterVelocity() < targetVelocity + lenience ))) {
                targetVelocity =(3.36048 * localizer.getDistanceToGoal()) + 1325.87065;
                shooter.regression(localizer.getDistanceToGoal());
            }

            // shoots
            stopper.setStopper(0.0);
            Actions.runBlocking(refreshAndWait);
            stopper.setStopper(0.8);

            intake.intakePower(0.0);

            // parks
            Actions.runBlocking(startToGarden);
            Actions.runBlocking(gardenToPark);
        }
        else {
            // goes over to the other side
            Actions.runBlocking(starttogarden);
            Actions.runBlocking(gardentoshoot);

            // updates limelight again
            limelight.getClosest();

            // checks if hive is up or down
            if (limelight.getTag() != null) {

                // refreshes the aim and flywheel power
                Actions.runBlocking(refresh);

                // waits until flywheel power is good enough
                targetVelocity =(3.36048 * localizer.getDistanceToGoal()) + 1325.87065;
                while (opModeIsActive() && !((shooter.getShooterVelocity() > targetVelocity - lenience ) && (shooter.getShooterVelocity() < targetVelocity + lenience ))) {
                    targetVelocity =(3.36048 * localizer.getDistanceToGoal()) + 1325.87065;
                    shooter.regression(localizer.getDistanceToGoal());
                }

                // shoots
                stopper.setStopper(0.0);
                Actions.runBlocking(refreshAndWait);
                stopper.setStopper(0.8);

                intake.intakePower(0.0);

                // parks
                Actions.runBlocking(shootToPark);
            }
            else {
                // goes back to the garden
                Actions.runBlocking(shoottogarden);

                // refreshes the aim and flywheel power
                Actions.runBlocking(refresh);

                // waits until flywheel power is good enough
                targetVelocity =(3.36048 * localizer.getDistanceToGoal()) + 1325.87065;
                while (opModeIsActive() && !((shooter.getShooterVelocity() > targetVelocity - lenience) && (shooter.getShooterVelocity() < targetVelocity + lenience ))) {
                    targetVelocity =(3.36048 * localizer.getDistanceToGoal()) + 1325.87065;
                    shooter.regression(localizer.getDistanceToGoal());
                }

                // shoots
                stopper.setStopper(0.0);
                Actions.runBlocking(refreshAndWait);
                stopper.setStopper(0.8);

                intake.intakePower(0.0);

                // parks
                Actions.runBlocking(gardenToPark);
            }
        }
    }
}
