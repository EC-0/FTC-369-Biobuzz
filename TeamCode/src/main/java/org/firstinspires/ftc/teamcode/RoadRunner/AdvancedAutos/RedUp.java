// add stopper one day

package org.firstinspires.ftc.teamcode.RoadRunner.AdvancedAutos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.RobotLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

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
    Limelight limelighter;
    Turret turret;
    Intake intake;
    RobotLocalizer robotlocalizer;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new MecanumDrive(hardwareMap, initialPose);
        limelighter = new Limelight(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        robotlocalizer = new RobotLocalizer(hardwareMap, initialPose, RobotLocalizer.allianceColor.RED);

        limelighter.switchPipeline(1);

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

        limelighter.getClosest(); // refreshes limelight stat

        if (limelighter.getTag() != null) {
            Actions.runBlocking(runWithTurret(startToGarden, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake())); // intakes garden pollen
            // shoots garden pollen
            new SleepAction(tiptime); // waits for potential tip

            limelighter.getClosest(); // refreshes limelight stat

            if (limelighter.getTag() != null) {
                Actions.runBlocking(runWithTurret(gardenToFlower, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake())); // intakes flower pollen
                Actions.runBlocking(runWithTurret(flowerToGarden, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake()));
                // shoots flower pollen
                Actions.runBlocking(runWithTurret(gardenToPark, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake()));
            }
            else {
                Actions.runBlocking(runWithTurret(gardenToFlower, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake())); // intakes flower pollen
                Actions.runBlocking(runWithTurret(flowerToShoot, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake()));

                limelighter.getClosest(); // refreshes limelight stat

                if (limelighter.getTag() != null) {
                    // shoots flower pollen
                    Actions.runBlocking(runWithTurret(shootToPark, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake()));
                }
                else {
                    Actions.runBlocking(runWithTurret(shootToGarden, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake()));
                    // shoots flower pollen
                    Actions.runBlocking(runWithTurret(gardenToPark, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake()));
                }
            }
        }
        else {
            Actions.runBlocking(runWithTurret(startToGarden, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake())); // intakes garden pollen
            Actions.runBlocking(runWithTurret(gardenToShoot, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake()));

            limelighter.getClosest(); // refreshes limelight stat

            if (limelighter.getTag() != null) {
                // shoots garden pollen
                new SleepAction(tiptime); // waits for potential tips
                Actions.runBlocking(runWithTurret(shootToFlower, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake())); // intakes flower pollen
                Actions.runBlocking(runWithTurret(flowerToShoot, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake()));

                limelighter.getClosest(); // refreshes limelight stat

                if (limelighter.getTag() != null) {
                    // shoots flower pollen
                    Actions.runBlocking(runWithTurret(shootToPark, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake()));
                }
                else {
                    Actions.runBlocking(runWithTurret(shootToGarden, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake()));;
                    // shoots flower pollen
                    Actions.runBlocking(runWithTurret(gardenToPark, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake()));;
                }
            }
            else {
                Actions.runBlocking(runWithTurret(shootToGarden, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake()));;
                // shoots pollen from garden
                Actions.runBlocking(runWithTurret(gardenToFlower, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake()));; // intakes flower pollen
                Actions.runBlocking(runWithTurret(flowerToGarden, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake()));
                // shoots pollen from flower
                Actions.runBlocking(runWithTurret(gardenToPark, runFlywheel(robotlocalizer.getDistanceToGoal()), aim(robotlocalizer.angleToGoal()), intake()));;
            }
        }
    }
    public static Action runWithTurret(Action path, Action flywheeler, Action aim, Action intake) {
        return new RaceAction(path, flywheeler, aim, intake);
    }

    public Action runFlywheel(double distance) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                turret.setTurretPower(distance);
                return false;
            }
        };
    }

    public Action aim(double botErrorDeg) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                turret.update(botErrorDeg);
                return false;
            }
        };
    }

    public Action intake() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.intakePower(0.5);
                return false;
            }
        };
    }
}
