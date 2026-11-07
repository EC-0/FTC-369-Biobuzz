package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF_Controller {

    private final double kP, kI, kD, kF;
    private double integral = 0, derivative;
    private double target = 0, lastTimeNano = 0, lastError = 0;
//    private double lastError = 0, target, error, output;
    private ElapsedTime timer = new ElapsedTime();

    public PIDF_Controller (double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        timer.reset();
    }

    public void setTarget(double target) {
        this.target = target;
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

        return (kP * error) + (kI * integral) + (kD * derivative) + kF;
    }
}
