package org.firstinspires.ftc.teamcode.TeleOpClasses;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.OdomLimelightTurret;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.RobotLocalizer;

@TeleOp(name = "Test TeleOp")
public class TestTeleop extends OpMode {
    MecanumDrive drive;
    Drivetrain drivetrain;
    Intake intake;
    RobotLocalizer localizer;
    OdomLimelightTurret odomLimelightTurret;

    private final Pose2d GOAL_TARGET = new Pose2d(new Vector2d(10, -10), Rotation2d.exp(0));
    private boolean turretLockEnabled = false;

    @Override
    public void init() {
        odomLimelightTurret = new OdomLimelightTurret(hardwareMap, RobotLocalizer.allianceColor.BLUE);
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        localizer = new RobotLocalizer(hardwareMap, new Pose2d(-72, -72, Math.toRadians(0)), RobotLocalizer.allianceColor.BLUE);
    }

    @Override
    public void loop() {
        localizer.updateBotPosition();
        Pose2d currentRobotPose = localizer.getBotPose();
        double botDistanceToGoal = localizer.getDistanceToGoal();

        drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.left_trigger, gamepad1.right_trigger);
        double botErrorDeg = 0.0;
        if (currentRobotPose != null) {
            double dx = GOAL_TARGET.position.x - currentRobotPose.position.x;
            double dy = GOAL_TARGET.position.y - currentRobotPose.position.y;
            double targetFieldAngle = Math.atan2(dy, dx);
            double robotHeading = currentRobotPose.heading.log();
            double targetRelativeAngle = AngleUnit.normalizeRadians(targetFieldAngle - robotHeading);
            botErrorDeg = Math.toDegrees(targetRelativeAngle) % 360;

            telemetry.addData("Hive Target X", GOAL_TARGET.position.x);
            telemetry.addData("Hive Target Y", GOAL_TARGET.position.y);
            telemetry.addData("Target Field Angle", Math.toDegrees(targetFieldAngle));
            telemetry.addData("Robot Heading", Math.toDegrees(robotHeading));
            telemetry.addData("Target Relative Angle", botErrorDeg);
        }

        odomLimelightTurret.setBotErrorDeg(botErrorDeg);

        // Feed vision target availability directly from the internal Limelight module
        boolean visionTargetVisible = odomLimelightTurret.limelight.limelighter.getLatestResult() != null
                && odomLimelightTurret.limelight.limelighter.getLatestResult().isValid();
        odomLimelightTurret.setLlHasTarget(visionTargetVisible);

        // Optional: Map gamepad2 right stick to act as manual override if touched
        odomLimelightTurret.setManualControl(gamepad2.right_stick_x);

        // 5. Gamepad 1 State Toggles
        if (gamepad1.rightBumperWasPressed()) {
            turretLockEnabled = !turretLockEnabled;
            if (turretLockEnabled) {
                // Switches to Assist Mode (Uses vision, drops back to Odom automatically if lost)
                odomLimelightTurret.setTurretMode(OdomLimelightTurret.Mode.LIMELIGHT_ASSIST_MODE);
            } else {
                odomLimelightTurret.setTurretMode(OdomLimelightTurret.Mode.LOCKED_TURRET_MODE);
            }
        }

        if (gamepad1.leftBumperWasPressed()) {
            odomLimelightTurret.turret.setShooterMode();
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
            odomLimelightTurret.turret.setTurretPower(botDistanceToGoal, 100);
        }
        if (gamepad1.xWasPressed()) {
            odomLimelightTurret.turret.setTurretPower(botDistanceToGoal, -100);
        }

        // Gamepad 2 Control Overrides
        if (gamepad2.leftBumperWasPressed()) {
            odomLimelightTurret.setTurretMode(OdomLimelightTurret.Mode.LIMELIGHT_BASIC_MODE);
        }
        if (gamepad2.rightBumperWasPressed()) {
            odomLimelightTurret.setTurretMode(OdomLimelightTurret.Mode.LOCKED_TURRET_MODE);
        }
        if (gamepad2.xWasPressed()) {
            odomLimelightTurret.setTurretMode(OdomLimelightTurret.Mode.MANUAL_RESET_MODE);
        }

        // Manual Reset Logic State Propagation
        if (gamepad2.x) {
            odomLimelightTurret.setManualResetHeld(true);
        }
        if (gamepad2.xWasReleased()) {
            odomLimelightTurret.setManualResetHeld(false);
        }

        // 6. RUN THE SUBSYSTEM ENGINE (CRITICAL FIX)
        odomLimelightTurret.update();

        // 7. Manage Shooter Flywheel Power separately
        odomLimelightTurret.turret.setTurretPower(botDistanceToGoal);

        // 8. Diagnostics Telemetry
        telemetry.addData("Turret Lock Status", turretLockEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Active Turret Mode", odomLimelightTurret.getTurretMode());
        telemetry.addData("Flywheel Power", odomLimelightTurret.turret.getFlyPower());
        telemetry.addData("Distance From Goal", botDistanceToGoal);
        telemetry.addData("Shooter Mode", odomLimelightTurret.turret.getShooterMode());

        if (currentRobotPose != null) {
            telemetry.addData("Robot X", currentRobotPose.position.x);
            telemetry.addData("Robot Y", currentRobotPose.position.y);
            telemetry.addData("Robot Heading Deg", Math.toDegrees(currentRobotPose.heading.log()));
        }
        telemetry.update();
    }
}
