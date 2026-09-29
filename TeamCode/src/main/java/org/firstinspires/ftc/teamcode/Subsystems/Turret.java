package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "turret")
public class Turret extends OpMode {
    DcMotorEx fly1;
    DcMotorEx fly2;
    double flyPower;
    @Override
    public void init() {
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        flyPower = 0;
    }

    public double setMotorManual(){
        if(gamepad1.a){
            flyPower += 25;
        }
        if(gamepad1.x){
            flyPower -= 25;
        }
        return flyPower;
    }
    @Override
    public void loop() {
        fly1.setPower(setMotorManual());
        fly2.setPower(setMotorManual());
    }
}
