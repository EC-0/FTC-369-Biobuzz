package org.firstinspires.ftc.teamcode.TeleOpClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.PIDF_Controller;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp(name = "TurretTestPID")
public class PIDTestTeleop extends OpMode {
    PIDF_Controller turretPID;
    DcMotorEx turret;
    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turretPID = new PIDF_Controller(1,0,0,0,0);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            turret.setPower(turretPID.calculate(turret.getCurrentPosition()));
        }
        else{
            turret.setPower(0);
        }
    }
}
