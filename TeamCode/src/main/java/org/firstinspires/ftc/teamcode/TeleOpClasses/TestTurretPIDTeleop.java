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

    public static double kp = 0.02;
    public static double kd =0;
    DcMotorEx turret;
    PIDF_Controller turretPID;
    FtcDashboard dash;
    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turretPID = new PIDF_Controller(kp,0,kd,0);
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

        packet.put("Turret pos", turret.getCurrentPosition());
        packet.put("Turret error", 450-turret.getCurrentPosition());

        // Send the packet to the dashboard browser
        dash.sendTelemetryPacket(packet);
        if(gamepad1.a){
            turret.setPower(turretPID.calculate(turret.getCurrentPosition()));
            turretPID.setTarget(450);
        }
        else{
            turret.setPower(0);
        }
    }
    
}
