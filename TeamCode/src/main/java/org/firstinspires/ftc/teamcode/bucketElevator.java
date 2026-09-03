package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.annotation.Target;

@TeleOp (name = "Elevator")
public class bucketElevator extends OpMode {
    double kP, kI, kD, elevatorPower;
    int desiredDistance = 1000;
    int tolerance = 5, targetPosition = 0;
    boolean tilted = false;
    DcMotor Elevator;
    Servo Left, Right;
    PID_Controller PID;

    @Override
    public void init() {
        // Change Name of HWM Device Names Later
        Elevator = hardwareMap.get(DcMotor.class, "Motor");
        Left = hardwareMap.get(Servo.class, "Servo1");
        Right = hardwareMap.get(Servo.class, "Servo2");

        PID = new PID_Controller(kP, kI, kD);

        Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // NOT GUARANTEED TO ACTUALLY BE THE LEFT ONE, CHECK WHILE TESTING!!!
        Left.setDirection(Servo.Direction.REVERSE);

        Elevator.setPower(0);
        Left.setPosition(0);
        Right.setPosition(0);
    }

    @Override
    public void loop() {

        // Detecting button inputs
        if (gamepad1.a) {
            targetPosition = desiredDistance;
            PID.setTarget(targetPosition);
            Left.setPosition(0.0);
            Right.setPosition(0.0);
            tilted = false;
        }
        else if (gamepad1.y) {
            targetPosition = 0;
            Left.setPosition(0.0);
            Right.setPosition(0.0);
            tilted = false;
        }

        // Tilting if reached top
        if (targetPosition == desiredDistance && Elevator.getCurrentPosition() > (desiredDistance - tolerance)) {
            Left.setPosition(0.8);
            Right.setPosition(0.8);
            tilted = true;
        }

        /* Waiting for the bucket to untilt and for the servos to set back to 0 before changing the PID
           target to 0 */
        if (targetPosition == 0 && !tilted && Left.getPosition() == 0.0 && Right.getPosition() == 0.0) {
            PID.setTarget(0);
        }

        //Applying the power
        elevatorPower = PID.calculate(Elevator.getCurrentPosition());
        Elevator.setPower(elevatorPower);
    }
}