package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "turret test encoder ticks")
public class TurretTest extends OpMode {
    DcMotorEx turret;
    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");

    }

    @Override
    public void loop() {

    }
}
