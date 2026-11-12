package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter{
    DcMotorEx fly1;

    private double flyPower = 0;
    private boolean useRegression = false;
    private double targetVelocity = 0;
    private long lastLoopTimeNs = 0;
    private double lastError;

    private double kV = 0.00035; // main feedforward for velocity
    private double kS = 0.044; //feedforward just for static friction
    private double kP = 0.005; //proportional
    private double kD = 0; //derivative


    public Shooter(HardwareMap hardwareMap){
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
    }

    public void setShooterMode() {
        useRegression = !useRegression;
    }
    public void setShooterPower(double distance) {
        if (useRegression) {
            regression(distance);
        } else {
            fly1.setVelocity(flyPower);
            // fly2.setVelocity(flyPower);
        }
    }
    public void setShooterPower(double distance, double increment) {
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
        setPIDTarget(targetVelocity);
        fly1.setPower(shooterPID(fly1.getVelocity()));
        // fly2.setVelocity(pid.calculate(fly2.getVelocity()));
    }
    public void setPIDTarget(double targetVelocity){
        this.targetVelocity = targetVelocity;
    }

    public double shooterPID(double currentVelocity){
        long nowNs = System.nanoTime();
        double dt = (nowNs - lastLoopTimeNs) / 1e9;
        lastLoopTimeNs = nowNs;
        dt = Math.max(dt, 1e-6);
        double error = targetVelocity - currentVelocity;
        double errorDerivative = (error - lastError) / dt;
        lastError = error;

        double feedforward = (kV * targetVelocity) + (kS * Math.signum(targetVelocity));
        double proportional = kP * error;
        double derivative = kD * errorDerivative;

        return clamp(feedforward + proportional + derivative, -1, 1);
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
    public double getFlyPower() {
        return flyPower;
    }

    public double getShooterVelocity(){
        return fly1.getVelocity();
    }

    public String getShooterMode(){
        return useRegression ? "Auto" : "Manual";
    }
}
