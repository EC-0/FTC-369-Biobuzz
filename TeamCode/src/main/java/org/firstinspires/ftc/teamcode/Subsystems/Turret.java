
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {
    //DcMotorEx fly2;
    DcMotorEx turret;
    PIDF_Controller turretPid;

    private final double totalTurnTicks = 1544;
    private final double totalDegrees = 320.0;
    private final double maxSafeTicks = 1530;
    private final double minSafeTicks = 15.0;
    private final double ticksPerDegree = totalTurnTicks / totalDegrees;

    private double currentPositionTicks;
    private double currentPositionDegrees;
    private double targetPositionTurretTicks;
    private double targetPositionTurretDegrees;

    private final double minDegrees = 15.0;
    private final double safeMinDegrees = 20.0;
    private final double safeMaxDegrees = 340.0;

    private double targetVelTicksPerSec = 0.0;
    private double lastTargetTicks = 0.0;
    private long lastTargetTimeNanos = 0;

    private double filteredTargetTicks = 0.0;
    private boolean filterInitialized = false;

    private static final double TARGET_ALPHA = 0.125;


    public Turret(HardwareMap hardwareMap) {

        //fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        //fly2.setDirection(DcMotorSimple.Direction.REVERSE);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretPid = new PIDF_Controller(0.005, 0, 0.0001, 0, 0);
    }

    public void setPowerRaw(double power) {
        turret.setPower(power);
    }

    public void resetPosition() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lastTargetTimeNanos = 0;
        lastTargetTicks = 0.0;
        targetVelTicksPerSec = 0.0;

        filterInitialized = false;
        filteredTargetTicks = 0.0;
    }
    public void limelightTracking(double tx) {
        currentPositionTicks = turret.getCurrentPosition();
        currentPositionDegrees = minDegrees + (currentPositionTicks / ticksPerDegree);
        filterInitialized = false;
        if (Math.abs(tx) < 0.35) {
            turret.setPower(0);
            return;
        }
        double targetDegrees = currentPositionDegrees + tx;
        targetDegrees = clamp(targetDegrees, safeMinDegrees, safeMaxDegrees);
        double targetTicks = (targetDegrees - minDegrees) * ticksPerDegree;
        turretPid.setTarget(targetTicks, 0);
        double outputPower = turretPid.calculate(currentPositionTicks);
        outputPower = clamp(outputPower, -0.45, 0.45);

        if (currentPositionTicks >= maxSafeTicks && outputPower > 0) {
            outputPower = 0;
        }
        if (currentPositionTicks <= minSafeTicks && outputPower < 0) {
            outputPower = 0;
        }

        turret.setPower(outputPower);
    }


    public void update(double botErrorDeg) {
        currentPositionTicks = turret.getCurrentPosition();
        currentPositionDegrees = normalize360(minDegrees + currentPositionTicks / ticksPerDegree);

        double desiredDegrees = normalize360(360.0 - botErrorDeg);
        targetPositionTurretDegrees = clamp(desiredDegrees, safeMinDegrees, safeMaxDegrees
        );

        targetPositionTurretTicks = (targetPositionTurretDegrees - minDegrees) * ticksPerDegree;

        if (!filterInitialized) {
            filteredTargetTicks = targetPositionTurretTicks;
            filterInitialized = true;
        } else {
            filteredTargetTicks += TARGET_ALPHA * (targetPositionTurretTicks - filteredTargetTicks);
        }

        updateTargetVelocity();
        turretPid.setTarget(filteredTargetTicks, targetVelTicksPerSec);
        turretPid.setTarget(filteredTargetTicks);
        double outputPower = turretPid.calculate(currentPositionTicks);
        outputPower = clamp(outputPower, -0.6, 0.6);
        if (currentPositionTicks >= maxSafeTicks && outputPower > 0) {
            outputPower = 0;
        }
        if (currentPositionTicks <= minSafeTicks && outputPower < 0) {
            outputPower = 0;
        }
        turret.setPower(outputPower);
    }

    private static double clamp(double x, double min, double max) {
        return Math.max(min, Math.min(max, x));
    }

    private static double normalize360(double degrees) {
        degrees %= 360.0;
        if (degrees < 0) {
            degrees += 360.0;
        }
        return degrees;
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
        if (dt <= 0 || dt > 0.1) {
            dt = 0.02;
        }
        targetVelTicksPerSec = (filteredTargetTicks - lastTargetTicks) / dt;
        lastTargetTicks = filteredTargetTicks;
    }

    public boolean isOnTarget(double degTolerance) {
        double tickDiff = Math.abs(turret.getCurrentPosition() - filteredTargetTicks);
        return (tickDiff / ticksPerDegree) < degTolerance;
    }

    public double getTargetVelocityTicksPerSec() {
        return targetVelTicksPerSec;
    }

    public double getTargetTicks() {
        return filteredTargetTicks;
    }

    public double getCurrentTicks() {
        return turret.getCurrentPosition();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret Current Ticks", currentPositionTicks);
        telemetry.addData("Turret Current Degrees", currentPositionDegrees);
        telemetry.addData("Turret Target Ticks", targetPositionTurretTicks);
        telemetry.addData("Turret Target Degrees", targetPositionTurretDegrees);
        telemetry.addData("Filtered Target Ticks", filteredTargetTicks);
        telemetry.addData("Target Velocity Ticks/Sec", targetVelTicksPerSec);
    }

    public int getCurrentPosition() {
        return turret.getCurrentPosition();
    }

    public void setPIDCoefficients(double kp, double ki, double kd, double ks, double kv){
        turretPid.setCoefficients(kp, ki, kd, ks, kv);
    }
}
