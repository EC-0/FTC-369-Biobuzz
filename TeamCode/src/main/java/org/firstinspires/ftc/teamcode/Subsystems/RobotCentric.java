package org.firstinspires.ftc.teamcode.Subsystems;

import android.view.contentcapture.DataShareWriteAdapter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@TeleOp(name = "RobotCentric")
public class RobotCentric extends OpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        if(gamepad1.a){
            frontLeft.setPower(0.8);
            frontRight.setPower(0.8);
            backLeft.setPower(0.8);
            backRight.setPower(0.8);
        }
        else if(gamepad1.b){
            frontLeft.setPower(-0.8);
            frontRight.setPower(-0.8);
            backLeft.setPower(-0.8);
            backRight.setPower(-0.8);
        }
        else{
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }
}
