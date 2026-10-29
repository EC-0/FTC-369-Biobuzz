package org.firstinspires.ftc.teamcode.TeleOpClasses;

import android.graphics.drawable.PictureDrawable;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.PIDF_Controller;

@TeleOp(name = "TestTurretPID")
public class TestTurretPIDTeleop extends OpMode {
    DcMotorEx turret;
    PIDF_Controller turretPID;
    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turretPID = new PIDF_Controller(2.5,0,2,0,0);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            turret.setPower(turretPID.calculate(turret.getCurrentPosition()));
            turretPID.setTarget(450);
        }
        else{
            turret.setPower(0);
        }
    }
}
