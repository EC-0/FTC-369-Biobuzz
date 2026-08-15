package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends OpMode {

    DcMotor intake;
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            intake.setPower(1.0);
        }
        else{
            intake.setPower(0.0);
        }
    }
}
