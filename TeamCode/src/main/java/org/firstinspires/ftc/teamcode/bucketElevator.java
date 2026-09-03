package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp (name = "Elevator")
public class bucketElevator extends OpMode {
    double kP, kI, kD, elevatorPower;
    int desiredHigherDistance = 1000;
    int targetPosition = 0;
    int tolerance = 5;
    String bucketStatus = "Ready";
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

        if (gamepad1.a) {
            Left.setPosition(0.8);
            Right.setPosition(0.8);

            PID.setTarget(desiredHigherDistance);
            targetPosition = desiredHigherDistance;
        }
        else if (gamepad1.x) {
            Left.setPosition(0);
            Right.setPosition(0);

            PID.setTarget(0);
            targetPosition = 0;
        }

        if (Elevator.getCurrentPosition() > targetPosition - tolerance && Elevator.getCurrentPosition() < targetPosition + tolerance) {
            if (targetPosition != 0) {
                Left.setPosition(0.8);
                Right.setPosition(0.8);
            }
        }

        elevatorPower = PID.calculate(Elevator.getCurrentPosition());
        if (Left.getPosition() == 0 && Right.getPosition() == 0) {
            Elevator.setPower(elevatorPower);
        }
    }
}