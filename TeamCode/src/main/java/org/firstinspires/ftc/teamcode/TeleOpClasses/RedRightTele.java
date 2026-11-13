package org.firstinspires.ftc.teamcode.TeleOpClasses;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Stopper;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.RobotLocalizer;

@TeleOp(name = "Red Right Teleop")
public class RedRightTele extends OpMode {
    private ElapsedTime tagLostTimer = new ElapsedTime();
    private boolean usingLimelight = false;

    MecanumDrive drive;
    Limelight limelight;
    Drivetrain drivetrain;
    Intake intake;
    Turret turret;
    Stopper stopper;
    RobotLocalizer localizer;

    private Pose2d GOAL_TARGET;

    private boolean turretLockEnabled = false;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);
        stopper = new Stopper(hardwareMap);
        //START POSE IS SET FOR RIGHT SIDE RED SIDE
        localizer = new RobotLocalizer(hardwareMap, new Pose2d(68, -68, Math.toRadians(0)), RobotLocalizer.allianceColor.RED);
        limelight.switchPipeline(1);
        GOAL_TARGET = new Pose2d(new Vector2d(localizer.GOAL_X, localizer.GOAL_Y), Rotation2d.exp(0));
    }

    @Override
    public void loop() {
        localizer.updateBotPosition();
        limelight.getClosest();

        Pose2d currentRobotPose = localizer.getBotPose();
        double botDistanceToGoal = localizer.getDistanceToGoal();

        drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.left_trigger,
                gamepad1.right_trigger);

        if (turretLockEnabled) {
            if (limelight.getTag() != null) {
                tagLostTimer.reset();
                usingLimelight = true;
            } else if (tagLostTimer.seconds() > 0.25) {
                usingLimelight = false;
            }

            if (usingLimelight) {
                double tx = limelight.getTx();
                turret.limelightTracking(tx);
                telemetry.addData("Turret locked", "Limelight Active");
            } else if (currentRobotPose != null) {
                turret.update(localizer.angleToGoal());

                telemetry.addData("Turret Lock", "ODOMETRY ACTIVE");
                telemetry.addData("Hive Target X", localizer.GOAL_X);
                telemetry.addData("Hive Target Y", localizer.GOAL_Y);
                localizer.printTelem(telemetry);
            }
        } else {
            usingLimelight = false;
            telemetry.addData("Turret Lock", "DISABLED");
            turret.setPowerRaw(0);
        }

        if (gamepad2.leftBumperWasPressed()) {
            turret.setShooterMode();
        }
        if (gamepad2.rightBumperWasPressed()) {
            turretLockEnabled = !turretLockEnabled;
        }
        if (gamepad1.x) {
            drivetrain.resetHeading();
        }
        if (gamepad1.a) {
            intake.intakePower(0.9);
            stopper.setStopper(.1);
        } else if (gamepad2.a) {
            intake.intakePower(0.0);
        } else if (gamepad2.b) {
            intake.intakePower(-.8);
        } else {
            intake.intakePower(0.5);
            stopper.setStopper(.8);
        }


        if (gamepad2.bWasPressed()) {
            turret.setTurretPower(botDistanceToGoal, 50);
        }
        if (gamepad2.xWasPressed()) {
            turret.setTurretPower(botDistanceToGoal, -50);
        }
        turret.setTurretPower(botDistanceToGoal);

        // gamepad 2 stuff
        if (gamepad2.xWasPressed()) {
            turret.setPowerRaw(0);
        }
        if (gamepad2.yWasPressed()) {
            currentRobotPose = new Pose2d(68, -68, 90);
        }

        telemetry.addData("flyPower", turret.getFlyPower());
        if (currentRobotPose != null) {
            telemetry.addData("Robot X", currentRobotPose.position.x);
            telemetry.addData("Robot Y", currentRobotPose.position.y);
            telemetry.addData("Robot Heading", Math.toDegrees(currentRobotPose.heading.log()));
        }

        telemetry.addData("distance from goal", botDistanceToGoal);
//        telemetry.addData("Distance from April Tag", limelight.getDistance());
        telemetry.addData("LIMELIGHT", limelight.getTag());
//        telemetry.addData("Shooter Mode", turret.getShooterMode());
        telemetry.update();
    }
}
