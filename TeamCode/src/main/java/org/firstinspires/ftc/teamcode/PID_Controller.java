package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID_Controller {

    /* Not currently sure if I even need Feedforward for the linear slide elevator
       check in with Russell or Ethan later. */
    private final double kP, kI, kD;
    private double integral, derivative;
    private double lastError, target, error, output;
    private ElapsedTime timer = new ElapsedTime();

    public PID_Controller (double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
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
        output = (kP * error) + (kI * integral) + (kD * derivative);
        lastError = error;

        return output;
    }
}
