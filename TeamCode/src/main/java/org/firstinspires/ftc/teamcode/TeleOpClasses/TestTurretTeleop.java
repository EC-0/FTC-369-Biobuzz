package org.firstinspires.ftc.teamcode.TeleOpClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp(name = "Turret Ticks")
public class TestTurretTeleop extends OpMode {
    DcMotorEx turret;
    Drivetrain drivetrain;
    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        drivetrain = new Drivetrain(hardwareMap);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addData("turret ticks", turret.getCurrentPosition());
        telemetry.update();
    }
}