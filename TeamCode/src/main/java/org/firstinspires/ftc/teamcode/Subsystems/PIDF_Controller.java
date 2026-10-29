package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF_Controller {

    private final double kP, kI, kD, kF, kV;
    private double integral, derivative;
    private double target, lastTimeNano, lastError = 0;
//    private double lastError = 0, target, error, output;
    private ElapsedTime timer = new ElapsedTime();

    public PIDF_Controller (double kP, double kI, double kD, double kF, double kV) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.kV = kV;
        timer.reset();
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double calculate(double current) {
        long now = System.nanoTime();

        if(lastTimeNano == 0){
            lastTimeNano = now;
            return 0;
        }
        double dt = now-lastTimeNano;

        double  error = target - current;
        derivative = (error - lastError) / dt;
        integral += error * dt;
        lastError = error;

        lastTimeNano  = timer.seconds();

        return (kP * error) + (kI * integral) + (kD * derivative) + kF;
    }
}
