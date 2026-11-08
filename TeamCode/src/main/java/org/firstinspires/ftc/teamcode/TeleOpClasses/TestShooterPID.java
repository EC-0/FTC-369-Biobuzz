package org.firstinspires.ftc.teamcode.TeleOpClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.PIDF_Controller;

@TeleOp (name = "turretPID")
public class TestShooterPID extends OpMode {
    DcMotorEx fly1;
//    DcMotorEx fly2;
    PIDF_Controller shooterPID;
    FtcDashboard dash;
    public static double kp;
    public static double kI;
    public static double kd;
    public static double kV;
    public static double kS;
    @Override
    public void init() {
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        //fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        shooterPID = new PIDF_Controller(1,0,0,0,0);
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turret pos", fly1.getVelocity());
        packet.put("Turret error", 1000-fly1.getVelocity());

        dash.sendTelemetryPacket(packet);
        if(gamepad1.dpad_left){
            fly1.setPower(-.3);
        }
        else if(gamepad1.dpad_right){
            fly1.setPower(.3);
        }
        else if(gamepad1.a){
            shooterPID.setTarget(1000);
            fly1.setVelocity(shooterPID.calculate(fly1.getVelocity()));
        }
        else{
            fly1.setPower(0);
        }

        if(gamepad1.dpadUpWasPressed()){
            kS+=.01;
        }
        if(gamepad1.dpadDownWasPressed()){
            kS-=.01;
        }

//        else turret.setPower(0);

        if(gamepad1.leftBumperWasPressed()){
            kV-=.0001;
        }
        if(gamepad1.rightBumperWasPressed()){
            kV+=.0001;
        }

        telemetry.addData("kS", kS);
        telemetry.addData("kV", kV);
        telemetry.update();
//        if(gamepad1.x) {
            shooterPID = new PIDF_Controller(kp, kI, kd, kS, kV);
//        }
    }

}
