package org.firstinspires.ftc.teamcode.RoadRunner.SimpleAutos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.BackgroundActions;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.PIDF_Controller;
import org.firstinspires.ftc.teamcode.Subsystems.RobotLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Stopper;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Autonomous (name = "BlueDown")
public class BlueDown extends LinearOpMode {

    double shootTime = 6, tipTime = 5, lenience = 50, targetVelocity;
    Shooter shooter;

    Limelight limelight;
    MecanumDrive drivetrain;
    Turret turret;
    Stopper stopper;
    Intake intake;
    RobotLocalizer localizer;
    BackgroundActions ba;
    PIDF_Controller pid;
    Pose2d initialPose = new Pose2d(62,10, Math.toRadians(270));
    Pose2d shoot = new Pose2d(-50, 50, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        limelight = new Limelight(hardwareMap);
        drivetrain = new MecanumDrive(hardwareMap, initialPose);
        turret = new Turret(hardwareMap);
        stopper = new Stopper(hardwareMap);
        intake = new Intake(hardwareMap);
        localizer = new RobotLocalizer(hardwareMap ,initialPose, RobotLocalizer.allianceColor.RED);
        ba = new BackgroundActions(turret, localizer, limelight, shooter);
        pid = new PIDF_Controller(13.5, 0.05 , 0.01, 20,-.009625);

        limelight.switchPipeline(0);

        Action startToPark = drivetrain.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(50,10), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(50,62), Math.toRadians(270))
                .build();

        Action startToShoot = drivetrain.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(50,10), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(50,50), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-50,50), Math.toRadians(270))
                .build();

        Action shootToStart = drivetrain.actionBuilder(shoot)
                .strafeToLinearHeading(new Vector2d(50,50), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(50,10), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-50,50), Math.toRadians(270))
                .build();

        Action shootToPark = drivetrain.actionBuilder(shoot)
                .strafeToLinearHeading(new Vector2d(50,50), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(50,62), Math.toRadians(270))
                .build();

        RaceAction getReady = new RaceAction(
                new SleepAction(3.0),
                ba.combinedActions()
        );

        RaceAction waitForTip = new RaceAction(
                new SleepAction(tipTime + shootTime),
                ba.getClosest(),
                ba.getTag()
        );

        RaceAction starttoshoot = new RaceAction(
                startToShoot,
                ba.combinedActions()
        );

        RaceAction shoottostart = new RaceAction(
                shootToStart,
                ba.combinedActions()
        );

        RaceAction refresh = new RaceAction(
                new SleepAction(1.0),
                ba.combinedActions()
        );

        RaceAction refreshAndWait = new RaceAction(
                new SleepAction(shootTime),
                ba.combinedActions()
        );

        waitForStart();

        // starts intaking and starts up the stopper
        stopper.setStopper(0.8);
        intake.intakePower(0.9);

        // has 3 seconds of setting up, by running the motor and aiming the turret
        Actions.runBlocking(getReady);

        // waits until either a certain amount of time passes or the robot spots the april tags
        Actions.runBlocking(waitForTip);

        // refreshes the limelight
        Actions.runBlocking(ba.getClosest());

        // checks if the hive is tipped or not
        if (limelight.getTag() != null) {

            // refreshes aim and flywheel power
            Actions.runBlocking(refresh);

            // waits until the flywheel power is correct
            targetVelocity =(3.36048 * localizer.getDistanceToGoal()) + 1325.87065;
            while (opModeIsActive() && !(shooter.getShooterVelocity() > targetVelocity - lenience)) {
                targetVelocity =(3.36048 * localizer.getDistanceToGoal()) + 1325.87065;
                shooter.regression(localizer.getDistanceToGoal());
            }

            // shoots
            stopper.setStopper(0.0);
            Actions.runBlocking(refreshAndWait);
            stopper.setStopper(0.8);

            intake.intakePower(0.0);

            // parks
            Actions.runBlocking(startToPark);
        }
        else {
            // goes to the shooting position
            Actions.runBlocking(starttoshoot);

            // refreshes the limelight
            ba.getClosest();

            // checks if the hive is tipped or not
            if (limelight.getTag() != null) {

                // updates flywheel power and aim
                Actions.runBlocking(refresh);

                // waits until flywheel power is good
                targetVelocity =(3.36048 * localizer.getDistanceToGoal()) + 1325.87065;
                while (opModeIsActive() && !(shooter.getShooterVelocity() > targetVelocity - lenience)) {
                    targetVelocity =(3.36048 * localizer.getDistanceToGoal()) + 1325.87065;
                    shooter.regression(localizer.getDistanceToGoal());
                }

                // shoots
                stopper.setStopper(0.0);
                Actions.runBlocking(refreshAndWait);
                stopper.setStopper(0.8);

                intake.intakePower(0.0);

                // parks
                Actions.runBlocking(shoottostart);
            }
            else {
                // goes back to start to shoot
                Actions.runBlocking(shoottostart);

                // updates flywheel power and aim
                Actions.runBlocking(refresh);

                // waits until flywheel power is good
                targetVelocity =(3.36048 * localizer.getDistanceToGoal()) + 1325.87065;
                while (opModeIsActive() && !(shooter.getShooterVelocity() > targetVelocity - lenience)) {
                    targetVelocity =(3.36048 * localizer.getDistanceToGoal()) + 1325.87065;
                    shooter.regression(localizer.getDistanceToGoal());
                }

                // shoots
                stopper.setStopper(0.0);
                Actions.runBlocking(refreshAndWait);
                stopper.setStopper(0.8);

                intake.intakePower(0.0);

                // parks
                Actions.runBlocking(startToPark);
            }
        }
    }
}
