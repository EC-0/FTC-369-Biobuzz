
package org.firstinspires.ftc.teamcode.TeleOpClasses;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.RobotLocalizer;

@TeleOp(name = "Test TeleOp")
public class TestTeleop extends OpMode {

    MecanumDrive drive;
    Limelight limelight;
    Drivetrain drivetrain;
    Intake intake;
    Turret turret;
    RobotLocalizer localizer;

    private final Pose2d GOAL_TARGET = new Pose2d(new Vector2d(10, -10), Rotation2d.exp(0)
    );
    private boolean turretLockEnabled = false;
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap, telemetry);
        localizer = new RobotLocalizer(hardwareMap, new Pose2d(-72, -72, Math.toRadians(0)), RobotLocalizer.allianceColor.BLUE);
        limelight.pipelineSwitch(0);
    }
    @Override
    public void loop() {
        localizer.updateBotPosition();
        Pose2d currentRobotPose = localizer.getBotPose();
        double botDistanceToGoal = localizer.getDistanceToGoal();

        drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.left_trigger, gamepad1.right_trigger);

        if (gamepad1.rightBumperWasPressed()) {
            turretLockEnabled = !turretLockEnabled;
        }
        if (turretLockEnabled && currentRobotPose != null) {
            double dx = GOAL_TARGET.position.x - currentRobotPose.position.x;
            double dy = GOAL_TARGET.position.y - currentRobotPose.position.y;
            double targetFieldAngle = Math.atan2(dy, dx);
            double robotHeading = currentRobotPose.heading.log();
            double targetRelativeAngle = targetFieldAngle - robotHeading;
            double botErrorDeg = Math.toDegrees(targetRelativeAngle);
            botErrorDeg = ((botErrorDeg + 180) % 360 + 360) % 360 - 180;
            turret.update(botErrorDeg);

            telemetry.addData("Turret Lock", "ENABLED");
            telemetry.addData("Hive Target X", GOAL_TARGET.position.x);
            telemetry.addData("Hive Target Y", GOAL_TARGET.position.y);
            telemetry.addData("Target Field Angle", Math.toDegrees(targetFieldAngle));
            telemetry.addData("Robot Heading", Math.toDegrees(robotHeading));
            telemetry.addData("Target Relative Angle", botErrorDeg);
        } else {
            telemetry.addData("Turret Lock", "DISABLED");
            turret.setPowerRaw(0);
        }
        if (gamepad1.leftBumperWasPressed()) {
            turret.setShooterMode();
        }
        if (gamepad1.y) {
            drivetrain.resetHeading();
        }
        if (gamepad1.a) {
            intake.intakePower(0.8);
        } else {
            intake.intakePower(0);
        }
        if (gamepad1.bWasPressed()) {
            turret.setTurretPower(botDistanceToGoal, 100);
        }
        if (gamepad1.xWasPressed()) {
            turret.setTurretPower(botDistanceToGoal, -100);
        }
        turret.setTurretPower(botDistanceToGoal);
        if (currentRobotPose != null) {
            telemetry.addData("Robot X", currentRobotPose.position.x);
            telemetry.addData("Robot Y", currentRobotPose.position.y);
            telemetry.addData("Robot Heading", Math.toDegrees(currentRobotPose.heading.log()));
            telemetry.addData("Robot Pose", currentRobotPose);
        }
        telemetry.addData("Distance to Goal", botDistanceToGoal);
        telemetry.addData("Turret Current Ticks", turret.getCurrentTicks());
        telemetry.addData("Turret Target Ticks", turret.getTargetTicks());
        telemetry.addData("Distance from AprilTag", limelight.distanceFromTag());
        turret.addTelemetry(telemetry);
        telemetry.update();
    }
}
