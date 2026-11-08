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
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Config
@TeleOp(name = "TestTurretPID")
public class TestTurretPIDTeleop extends OpMode {

    public static double kp = 0.0;
    public static double kI;
    public static double kS;
    public static double kV;
    public static double kd = 0.0;
    Turret turret;
    PIDF_Controller turretPID;
    FtcDashboard dash;
    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        turretPID = new PIDF_Controller(kp,0,kd,0,0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dash = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Turret pos", turret.getCurrentPosition());
//        packet.put("Turret error", 570-turret.getCurrentPosition());

        dash.sendTelemetryPacket(packet);
        if(gamepad1.dpad_left){
            turret.setPowerRaw(-.3);
        }
        else if(gamepad1.dpad_right){
            turret.setPowerRaw(.3);
        }
        else if(gamepad1.a){
//            turretPID.setTarget(1000);
            turret.update(90);
        }
        else{
            turret.setPowerRaw(0);
        }

        if(gamepad1.dpadUpWasPressed()){
            kS+=.01;
        }
        if(gamepad1.dpadDownWasPressed()){
            kS-=.01;
        }

//        else turret.setPower(0);

        if(gamepad1.leftBumperWasPressed()){
            kI-=.001;
        }
        if(gamepad1.rightBumperWasPressed()){
            kI+=.001;
        }

        telemetry.addData("kS", kS);
        telemetry.addData("kV", kV);
        telemetry.update();
//        if(gamepad1.x) {
            turret.setPIDCoefficients(kp, kI, kd, kS, kV);
//        }
    }
    
}
