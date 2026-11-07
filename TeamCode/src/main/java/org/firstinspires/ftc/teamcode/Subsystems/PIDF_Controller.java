package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF_Controller {

    private final double kP, kI, kD, kS, kV;


    private double integral = 0, derivative;
    private double target = 0, targetVel = 0, lastTimeNano = 0, lastError = 0;
//    private double lastError = 0, target, error, output;
    private ElapsedTime timer = new ElapsedTime();

    public PIDF_Controller (double kP, double kI, double kD, double kV, double kS) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        timer.reset();
    }



    public void setTarget(double target) {
        this.target = target;
    }

    public void setTarget(double target, double targetVel) {
        this.target = target;
        this.targetVel = targetVel;
    }
    public double calculate(double current) {
        long now = System.nanoTime();

        if(lastTimeNano == 0){
            lastTimeNano = now;
            lastError = target-current ;
            return 0;
        }
        double dt = (now-lastTimeNano)/1e9;

        double  error = target - current;
        derivative = (error - lastError) / dt;
        integral += error * dt;
        lastError = error;

        lastTimeNano  = now;

        double ff =0;

        // If target is still changing, use target velocity direction
        if (Math.abs(targetVel) > 1e-6) {
            ff += kS * Math.signum(targetVel);
        }
        // Otherwise, if we are still not at the target, push in error direction
        else if (Math.abs(error) > 1.0) {  // 1 tick deadband
            ff += kS * Math.signum(error);
        }

        // Velocity feedforward
        ff += kV * targetVel;


        return (kP * error) + (kI * integral) + (kD * derivative) + ff;
    }
}
