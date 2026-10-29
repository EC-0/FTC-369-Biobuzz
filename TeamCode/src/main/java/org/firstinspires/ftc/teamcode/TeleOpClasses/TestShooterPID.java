package org.firstinspires.ftc.teamcode.TeleOpClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.PIDF_Controller;

@TeleOp (name = "turretPID")
public class TestShooterPID extends OpMode {
    DcMotorEx fly1;
    DcMotorEx fly2;
    PIDF_Controller shooterPID;
    @Override
    public void init() {
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        shooterPID = new PIDF_Controller(1,0,0,0);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            fly1.setVelocity(shooterPID.calculate(fly1.getVelocity()));
            fly2.setVelocity(shooterPID.calculate(fly2.getVelocity()));
        }
        else{
            fly1.setVelocity(0);
            fly2.setVelocity(0);
        }
        telemetry.addData("current velocity", fly1.getVelocity());
        telemetry.addData("current velocity", fly2.getVelocity());
        telemetry.addData("target velocity", 700);
    }


}
