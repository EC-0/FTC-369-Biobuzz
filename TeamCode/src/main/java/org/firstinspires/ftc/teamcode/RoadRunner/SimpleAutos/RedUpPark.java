package org.firstinspires.ftc.teamcode.RoadRunner.SimpleAutos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.AdvancedAutos.Background;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.RobotLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.Stopper;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Autonomous(name = "Red Autonmous Tipped Up (Park)")
public class RedUpPark extends LinearOpMode {
    Intake intake;
    Turret turret;
    Limelight limelighter;

    MecanumDrive drivetrain;
    Background background;
    RobotLocalizer robotlocalizer;
    Stopper stopper;
    Pose2d initialPose = new Pose2d(62,-12,Math.toRadians(270));
    Pose2d garden = new Pose2d(62,-57, Math.toRadians(270));
    Pose2d shoot = new Pose2d(-55,-45, Math.toRadians(270));
    Pose2d flower = new Pose2d(24,-56, Math.toRadians(270));
    Pose2d park = new Pose2d(-20,-55, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain = new MecanumDrive(hardwareMap, initialPose);
        limelighter = new Limelight(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        stopper = new Stopper(hardwareMap);

        background = new Background(turret, intake);

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

        Action gardenToPark = drivetrain.actionBuilder(garden)
                .strafeToLinearHeading(new Vector2d(-25,-40), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-20,-55), Math.toRadians(270))
                .build();

        waitForStart();

        Actions.runBlocking(startToGarden);
        Actions.runBlocking(gardenToPark);
    }
}
