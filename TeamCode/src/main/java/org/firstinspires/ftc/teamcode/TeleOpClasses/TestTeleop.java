package org.firstinspires.ftc.teamcode.TeleOpClasses;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.RobotLocalizer;

@TeleOp (name = "Test TeleOp")
public class TestTeleop extends OpMode {
    MecanumDrive drive;
    Limelight limelight;
    Drivetrain drivetrain;
    Intake intake;
    Turret turret;
    RobotLocalizer localizer;
    private final Pose2d GOAL_TARGET = new Pose2d(new Vector2d(10, -10), Rotation2d.exp(0));

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);
        localizer = new RobotLocalizer(hardwareMap, new Pose2d(-72, -72, Math.toRadians(0)), RobotLocalizer.allianceColor.BLUE);
        limelight.switchPipeline(0);
    }

    @Override
    public void loop() {
        localizer.updateBotPosition();

        Pose2d currentRobotPose = localizer.getBotPose();
        double botDistanceToGoal = localizer.getDistanceToGoal();




        drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.left_trigger,
                gamepad1.right_trigger);


        if(gamepad1.right_bumper) {
            if (currentRobotPose != null) {
                turret.turretTrackPosition(GOAL_TARGET, currentRobotPose);
            }
        }
        if(gamepad1.leftBumperWasPressed()){
            turret.setShooterMode();
        }

        if (gamepad1.y) {
            drivetrain.resetHeading();
        }

        if (gamepad1.a) {
             intake.intakePower(0.8);
        }
        else{
            intake.intakePower(0);
        }

        if(gamepad1.bWasPressed()){
            turret.setTurretPower(botDistanceToGoal, 100);
        }
        if(gamepad1.xWasPressed()){
            turret.setTurretPower(botDistanceToGoal, -100);
        }
        turret.setTurretPower(botDistanceToGoal);

        telemetry.addData("flyPower", turret.getFlyPower());
        telemetry.addData("current robot positionX", currentRobotPose.position.x);
        telemetry.addData("current robot positionY", currentRobotPose.position.y);
        telemetry.addData("current robot positionTheta", Math.toDegrees(currentRobotPose.heading.log()));
        telemetry.addData("distance", botDistanceToGoal);
        telemetry.addData("Distance from April Tag", limelight.getDistance());

        telemetry.addData("Shooter Mode", turret.getShooterMode());
        telemetry.update();

        limelight.getClosest();

        /*
        else if(gamepad1.bWasPressed()){
            turret.regression(limelight.distanceFromTag());
        }
        */



    }
}
