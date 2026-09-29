package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "turret")
public class Turret extends OpMode {
    DcMotor intake;
    DcMotorEx fly1;
    DcMotorEx fly2;
    double flyPower;
    @Override
    public void init() {
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        fly2.setDirection(DcMotorSimple.Direction.REVERSE);
        flyPower = 0;
    }


    @Override
    public void loop() {
        if(gamepad1.b){
            intake.setPower(0.8);
        } else {
            intake.setPower(0);
        }
        if(gamepad1.aWasPressed()){
            flyPower += 100;
        }
        if(gamepad1.xWasPressed()){
            flyPower -= 100;
        }
        telemetry.addData("flyPower", flyPower);
        fly1.setVelocity(flyPower);
        telemetry.addData("fly1", fly1.getVelocity());
        fly2.setVelocity(flyPower);
        telemetry.addData("fly2", fly2.getVelocity());
    }
}
