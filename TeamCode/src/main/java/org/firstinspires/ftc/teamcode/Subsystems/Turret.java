package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Turret {

    DcMotorEx fly1;
    DcMotorEx fly2;
    DcMotorEx turret;
    PIDF_Controller pid;

    // Custom PID for the turret positioning loop since motor is in RUN_WITHOUT_ENCODER
    PIDF_Controller turretPid;

    double flyPower;
    boolean useRegression;
    private double totalTurnTicks = 853.0;
    private double totalDegrees = 320.0;
    private double maxSafeTicks = 840.0;
    private double minSafeTicks = 15.0;
    private double ticksPerDegree = totalTurnTicks / totalDegrees;
    private double currentPositionTicks;
    private double currentPositionDegrees;
    private double targetPositionTurretTicks;
    private double targetPositionTurretDegrees;
    private double minDegrees = 20;
    private double maxDegrees = 340;
    private double safeMinDegrees = 25;
    private double safeMaxDegrees = 335;
    private boolean inDeadZone;
    private double targetVelTicksPerSec = 0.0;
    private double lastTargetTicks = 0.0;
    private long lastTargetTimeNanos = 0;
    private double filteredTargetTicks = 0.0;
    private boolean filterInitialized = false;
    private static final double TARGET_ALPHA = 0.125;

    public Turret(HardwareMap hardwareMap) {
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        fly2.setDirection(DcMotorSimple.Direction.REVERSE);
        flyPower = 0;

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inDeadZone = true;

        pid = new PIDF_Controller(1, 0, 0, 0);

        // Initialize the tracking loop PID controller. Tune these coefficients.
        turretPid = new PIDF_Controller(0.005, 0, 0, 0);
    }

    public void setShooterMode() {
        useRegression = !useRegression;
    }

    public void setTurretPower(double distance) {
        if (useRegression)
            regression(distance);
        else {
            fly1.setVelocity(flyPower);
            fly2.setVelocity(flyPower);
        }
    }

    public void setTurretPower(double distance, double increment) {
        if (useRegression)
            regression(distance);
        else {
            flyPower += increment;
            fly1.setVelocity(flyPower);
            fly2.setVelocity(flyPower);
        }
    }

    public void regression(double distanceToHive) {
        double targetVelocity = 10.31616 * distanceToHive + 945.07026;
        pid.setTarget(targetVelocity);
        fly1.setVelocity(pid.calculate(fly1.getVelocity()));
        fly2.setVelocity(pid.calculate(fly2.getVelocity()));
    }

    public void setPowerRaw(double p) {
        turret.setPower(p);
    }

    public void resetPosition() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastTargetTimeNanos = 0;
        lastTargetTicks = 0.0;
        targetVelTicksPerSec = 0.0;
        filterInitialized = false;
        filteredTargetTicks = 0.0;
    }

    public void update(double botErrorDeg) {
        currentPositionTicks = turret.getCurrentPosition();
        currentPositionDegrees = normalize360(minDegrees + (-currentPositionTicks / ticksPerDegree));

        targetPositionTurretDegrees = clamp(normalize360(360 - botErrorDeg), safeMinDegrees, safeMaxDegrees);
        targetPositionTurretTicks = -(targetPositionTurretDegrees - minDegrees) * ticksPerDegree;

        if (!filterInitialized) {
            filteredTargetTicks = targetPositionTurretTicks;
            filterInitialized = true;
        } else {
            filteredTargetTicks = filteredTargetTicks + TARGET_ALPHA * (targetPositionTurretTicks - filteredTargetTicks);
        }

        updateTargetVelocity();

        turretPid.setTarget(filteredTargetTicks);
        double outputPower = turretPid.calculate(currentPositionTicks);

        outputPower = clamp(outputPower, -0.6, 0.6);
        turret.setPower(outputPower);
    }

    private static double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    private static double normalize360(double deg) {
        deg = deg % 360;
        if (deg < 0) deg += 360;
        return deg;
    }

    public double getTargetVelocityTicksPerSec() {
        return targetVelTicksPerSec;
    }

    private void updateTargetVelocity() {
        long now = System.nanoTime();

        if (lastTargetTimeNanos == 0) {
            lastTargetTimeNanos = now;
            lastTargetTicks = filteredTargetTicks;
            targetVelTicksPerSec = 0.0;
            return;
        }

        double dt = (now - lastTargetTimeNanos) / 1e9;
        lastTargetTimeNanos = now;

        if (dt <= 0 || dt > 0.1) dt = 0.02;

        targetVelTicksPerSec = (filteredTargetTicks - lastTargetTicks) / dt;
        lastTargetTicks = filteredTargetTicks;
    }

    public boolean isOnTarget(double degTolerance) {
        double tickDiff = Math.abs(turret.getCurrentPosition() - targetPositionTurretTicks);
        return (tickDiff / ticksPerDegree) < degTolerance;
    }

    public double getTargetTicks() {
        return filteredTargetTicks;
    }

    public double getCurrentTicks() {
        return turret.getCurrentPosition();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("curr ticks", currentPositionTicks);
        telemetry.addData("curr deg", currentPositionDegrees);
        telemetry.addLine("---------------");
        telemetry.addData("target ticks", targetPositionTurretTicks);
        telemetry.addData("target deg", targetPositionTurretDegrees);
        telemetry.addLine("---------------");
    }
}
