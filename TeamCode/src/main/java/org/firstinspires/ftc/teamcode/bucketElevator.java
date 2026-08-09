package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/* WARNING!!! THIS CODE WAS MADE VERY EARLY WITHOUT MUCH CONTEXT ON THE DETAILS OF THE ROBOT
   THE LOGIC AND THE WAY I HANDLED USING PIDF COULD BE VERY STUPID!! */
@TeleOp (name = "Elevator")
public class bucketElevator extends OpMode {
    /* NOTES TO REMEMBER FOR LATER:
       - add a constructor and remove the teleop to make this implementable into the final product in the future
       - desiredDistance is probably not 1000, so uh change that eventually
       - calibrate the PID values
       - there might be a problem with the gamepad input registering multiple times in a second,
         so that might mess up the code
       */
    double kP, kI, kD, elevatorPower;
    int desiredDistance = 1000;
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

        if (gamepad1.a && bucketStatus.equals("Ready")) {
            bucketStatus = "Raising";
            PID.setTarget(desiredDistance);
        }
        else if (gamepad1.a && bucketStatus.equals("Dumping")) {
            bucketStatus = "Lowering";
        }

        if (bucketStatus.equals("Raising")) {

            if (Elevator.getCurrentPosition() >= desiredDistance) {
                Left.setPosition(0.9);
                Right.setPosition(0.9);
                bucketStatus = "Dumping";
            }
        }

        if (bucketStatus.equals("Lowering")) {
            if (Left.getPosition() != 0.0 && Right.getPosition() != 0.0) {
                Left.setPosition(0.0);
                Right.setPosition(0.0);
            }

            else if (Left.getPosition() == 0.0 && Right.getPosition() == 0.0) {
                PID.setTarget(0);
            }

            if (Elevator.getCurrentPosition() <= 1) {
                bucketStatus = "Ready";
            }
        }

        elevatorPower = PID.calculate(Elevator.getCurrentPosition());
        Elevator.setPower(elevatorPower);

        telemetry.addData("Current Bucket Status: ", bucketStatus);
        telemetry.update();
    }
}