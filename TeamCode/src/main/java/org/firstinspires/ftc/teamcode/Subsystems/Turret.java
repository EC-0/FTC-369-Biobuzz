
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {
    DcMotorEx fly1;
    //DcMotorEx fly2;
    DcMotorEx turret;

    PIDF_Controller pid;
    PIDF_Controller turretPid;

    private double flyPower;
    private boolean useRegression;

    private final double totalTurnTicks = 2933;
    private final double totalDegrees = 320.0;
    private final double maxSafeTicks = 2870;
    private final double minSafeTicks = 15.0;
    private final double ticksPerDegree = totalTurnTicks / totalDegrees;

    private double currentPositionTicks;
    private double currentPositionDegrees;
    private double targetPositionTurretTicks;
    private double targetPositionTurretDegrees;

    private final double minDegrees = 20.0;
    private final double safeMinDegrees = 20.0;
    private final double safeMaxDegrees = 330.0;

    private double targetVelTicksPerSec = 0.0;
    private double lastTargetTicks = 0.0;
    private long lastTargetTimeNanos = 0;

    private double filteredTargetTicks = 0.0;
    private boolean filterInitialized = false;

    private static final double TARGET_ALPHA = 0.125;


    public Turret(HardwareMap hardwareMap) {

        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        //fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        //fly2.setDirection(DcMotorSimple.Direction.REVERSE);
        flyPower = 0;

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new PIDF_Controller(13.5, 0.05 , 0.01, 20,-.009625);
        turretPid = new PIDF_Controller(0.01, 0, 0.0001, 0, 0);
    }


    public void setShooterMode() {
        useRegression = !useRegression;
    }

    public void setTurretPower(double distance) {
        if (useRegression) {
            regression(distance);
        } else {
            fly1.setVelocity(flyPower);
           // fly2.setVelocity(flyPower);
        }
    }

    public void setTurretPower(double distance, double increment) {
        if (useRegression) {
            regression(distance);
        } else {
            flyPower += increment;
            fly1.setVelocity(flyPower);
           // fly2.setVelocity(flyPower);
        }
    }

    public void regression(double distanceToHive) {
        //double targetVelocity = 10.31616 * distanceToHive + 945.07026;
        double targetVelocity = (-2.78251 * distanceToHive) + 1720.39128;
        pid.setTarget(targetVelocity);
        fly1.setVelocity(pid.calculate(fly1.getVelocity()));
       // fly2.setVelocity(pid.calculate(fly2.getVelocity()));
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
        currentPositionDegrees = normalize360(minDegrees + currentPositionTicks / ticksPerDegree);

        filteredTargetTicks = currentPositionTicks;
        filterInitialized = true;

        if (Math.abs(tx) < 1.5) {
            turret.setPower(0);
            return;
        }

        double kP_limelight = 0.015; // Flip to -tx if it turns away from the tag
        double outputPower = tx * kP_limelight;
        outputPower = clamp(outputPower, -0.35, 0.35);

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

    private static double clamp(double value, double min, double max) {
        if (value < min) {
            return min;
        }
        else if (value > max) {
            return max;
        }
        return value;
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

    public double getFlyPower() {
        return flyPower;
    }

    public String getShooterMode(){
        return useRegression ? "Auto" : "Manual";
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
        turretPid = new PIDF_Controller(kp, ki, kd, ks, kv);
    }
}
