package org.firstinspires.ftc.teamcode.TeleOpClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.PIDF_Controller;
@Config
@TeleOp (name = "shooterPID")
public class TestShooterPID extends OpMode {
    DcMotorEx fly1;
    DcMotor intake;
    PIDF_Controller shooterPID;
    FtcDashboard dash;
    public static double kp = 1;
    public static double kI = 0;
    public static double kd = 0;
    public static double kV = 0;
    public static double kS = 0;
    @Override
    public void init() {
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        //fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        shooterPID = new PIDF_Controller(.1,0,0,0,0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dash = FtcDashboard.getInstance();
        intake = hardwareMap.get(DcMotor.class, "intake");


    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turret pos", fly1.getVelocity());
        packet.put("Turret error", 1000-fly1.getVelocity());

        dash.sendTelemetryPacket(packet);
        if(gamepad1.a){
            intake.setPower(0.8);
        }
        if(gamepad1.dpad_left){
            fly1.setPower(-.3);
        }
        else if(gamepad1.dpad_right){
            fly1.setPower(.3);
        }
        else if(gamepad1.a){
            shooterPID.setTarget(1000);
            fly1.setVelocity(shooterPID.calculate(fly1.getVelocity()));
            telemetry.addLine("its doin the velocity thing");
        }
//        else{
//            fly1.setPower(0);
//        }

        if(gamepad1.dpadUpWasPressed()){
            kp+=.5;
        }
        if(gamepad1.dpadDownWasPressed()){
            kp-=.5;
        }

//        else turret.setPower(0);

        if(gamepad1.leftBumperWasPressed()){
            kd-=.01;
        }
        if(gamepad1.rightBumperWasPressed()){
            kd+=.01;
        }
        if(gamepad1.bWasPressed()){
            kI-=.01;
        }
        if(gamepad1.yWasPressed()){
            kI+=.01;
        }
        telemetry.addData("kp", kp);
        telemetry.addData("kd", kd);
        telemetry.addData("ki", kI);
        telemetry.update();
//        if(gamepad1.x) {
            shooterPID.setCoefficients(kp, kI, kd, kS, kV);
//        }
    }

}
