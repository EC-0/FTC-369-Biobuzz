package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    Drivetrain drive;
    RobotLocalizer localizer;
    Limelight limelight;
    Intake intake;
    Shooter outtake;
    Turret turret;
    Shooter shooter;
    Stopper stopper;
    RobotLocalizer.allianceColor allianceColor;
    Telemetry telemetry;
    FtcDashboard dash;

    Gamepad gamepad1;
    Gamepad gamepad2;
    private ElapsedTime tagLostTimer = new ElapsedTime();
    private boolean usingLimelight = false;
    private boolean turretLockEnabled = false;

    public Robot(Telemetry telemetry, HardwareMap hardwareMap, RobotLocalizer.allianceColor allianceColor, Pose2d startPose, Gamepad gamepad1, Gamepad gamepad2){
        drive = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        limelight = new Limelight(hardwareMap);
        stopper = new Stopper(hardwareMap);
        this.allianceColor = allianceColor;
        dash = FtcDashboard.getInstance();
        localizer = new RobotLocalizer(hardwareMap, startPose, allianceColor);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        limelight.switchPipeline(allianceColor == RobotLocalizer.allianceColor.BLUE ? 0: 1);
        this.telemetry = telemetry;
    }
    public void start(){

    }
    public void update(){
        TelemetryPacket packet = new TelemetryPacket();

        localizer.updateBotPosition();
        limelight.getClosest();

        Pose2d currentBotPose = localizer.getBotPose();
        double distanceToGoal = localizer.getDistanceToGoal();

        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.left_trigger,
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
            } else if (currentBotPose != null) {
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
            shooter.setShooterMode();
        }
        if (gamepad2.rightBumperWasPressed()) {
            turretLockEnabled = !turretLockEnabled;
        }
        if (gamepad1.x) {
            drive.resetHeading();
        }
        if (gamepad1.a) {
            intake.intakePower(0.9);
            stopper.setStopper(.1);
        } else if (gamepad2.a) {
            intake.intakePower(0.6);
            stopper.setStopper(.4);
        } else if (gamepad2.b) {
            intake.intakePower(-.8);
        } else {
            intake.intakePower(0.0);
            stopper.setStopper(.4);
        }

        if (gamepad2.bWasPressed()) {
            shooter.setShooterPower(distanceToGoal, 50);
        }
        if (gamepad2.xWasPressed()) {
            shooter.setShooterPower(distanceToGoal, -50);
        }
        shooter.setShooterPower(distanceToGoal);

        // gamepad 2 stuff
        if (gamepad2.xWasPressed()) {
            turret.setPowerRaw(0);
        }
        if (gamepad2.yWasPressed()) {
            currentBotPose = new Pose2d(68, -68, 90);
        }

        telemetry.addData("flyPower", shooter.getFlyPower());
        if (currentBotPose != null) {
            telemetry.addData("Robot X", currentBotPose.position.x);
            telemetry.addData("Robot Y", currentBotPose.position.y);
            telemetry.addData("Robot Heading", Math.toDegrees(currentBotPose.heading.log()));
        }

        packet.put("Turret Actual Velocity", shooter.getShooterVelocity());
        packet.put("Turret Desired Power", 3.36048 * distanceToGoal +m1325.87065);

        dash.sendTelemetryPacket(packet);
        telemetry.addData("distance from goal", distanceToGoal);
//        telemetry.addData("Distance from April Tag", limelight.getDistance());
        telemetry.addData("LIMELIGHT", limelight.getTag());
//        telemetry.addData("Shooter Mode", turret.getShooterMode());
        telemetry.update();

    }
}
