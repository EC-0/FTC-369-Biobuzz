package org.firstinspires.ftc.teamcode.TeleOpClasses;

import android.graphics.drawable.PictureDrawable;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Subsystems.PIDF_Controller;

@Config
@TeleOp(name = "TestTurretPID")
public class TestTurretPIDTeleop extends OpMode {

    public static double kp = 0.01;
    public static double ki =0;
    public static double kd =0.0000;
    public static double kf =540;
    DcMotorEx turret;
    PIDF_Controller turretPID;
    FtcDashboard dash;
    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "fly1");
        turretPID = new PIDF_Controller(kp,ki,kd,kf);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dash = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Turret pos", turret.getVelocity());
        packet.put("Turret error", 1300-turret.getVelocity());

        // Send the packet to the dashboard browser
        dash.sendTelemetryPacket(packet);

        /*if(gamepad1.dpad_left)turret .setPower(-.3);
        else if(gamepad1.dpad_right)turret.setPower(.3);
        else*/  if(gamepad1.a){
            turretPID.setTarget(1300);
            turret.setVelocity(turretPID.calculate(turret.getVelocity()));
        }
        else{
            turret.setPower(0);
        }

        if(gamepad1.dpadUpWasPressed())kp+=.1;
        if(gamepad1.dpadDownWasPressed())kp-=.1;

//        else turret.setPower(0);

        if(gamepad1.leftBumperWasPressed())kd-=.001;
        if(gamepad1.rightBumperWasPressed())kd+=.001;

        telemetry.addData("kp", kp);
        telemetry.addData("kd", kd);
        telemetry.update();
        if(gamepad1.x) {
            turretPID = new PIDF_Controller(kp, ki, kd, kf);
        }
    }
    
}
