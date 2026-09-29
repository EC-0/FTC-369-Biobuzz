package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF_Controller {

    private final double kP, kI, kD, kF;
    private double integral, derivative;
    private double lastError, target, error, output;
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
        timer.reset();

        error = target - current;
        derivative = (error - lastError) / timer.seconds();
        integral += error * timer.seconds();
        output = (kP * error) + (kI * integral) + (kD * derivative) + kF;
        lastError = error;

        return output;
    }
}
