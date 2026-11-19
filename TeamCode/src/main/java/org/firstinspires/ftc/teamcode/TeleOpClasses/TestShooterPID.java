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
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@Config
@TeleOp (name = "shooterPID")
public class TestShooterPID extends OpMode {
    Shooter shooter;
    DcMotor intake;
    PIDF_Controller shooterPID;
    FtcDashboard dash;
    public static double kp = 0.0;//13.5;
    public static double kI = 0.0;//0.05;
    public static double kd = 0.0;//0.01;
    public static double kV = 0.0;
    public static double kS = 0.0;
    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        //fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        shooterPID = new PIDF_Controller(kp,kI,kd,kV,kS);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dash = FtcDashboard.getInstance();
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turret pos", shooter.getShooterVelocity());
        packet.put("Turret error", 1000-shooter.getShooterVelocity());

        dash.sendTelemetryPacket(packet);
        intake.setPower(gamepad1.right_stick_x);
        /*if(gamepad1.dpad_left){
            shooter.setShooterPower(-.3);
        }
        else if(gamepad1.dpad_right){
            shooter.setShooterPower(-.3);
        }
        else if(gamepad1.bWasPressed()){
            shooter.setShooterPower(-.3);
        }
        else*/
        if(gamepad1.a){
            shooter.setPIDTarget(1000);
            shooter.setVelocity(shooter.shooterPID(shooter.getShooterVelocity()));
            telemetry.addLine("its doin the velocity thing");
        }
        else{
            shooter.setVelocity(0);
        }

        if(gamepad1.dpadUpWasPressed()){
            kp+=.01;
        }
        if(gamepad1.dpadDownWasPressed()){
            kp-=.01;
        }

//        else turret.setPower(0);

        if(gamepad1.leftBumperWasPressed()){
            kd-=.01;
        }
        if(gamepad1.rightBumperWasPressed()){
            kd+=.01;
        }
//        if(gamepad1.bWasPressed()){
//            kI-=.01;
//        }
//        if(gamepad1.yWasPressed()){
//            kI+=.01;
//        }
        telemetry.addData("kP", kp);
        telemetry.addData("kD ", kd);
        telemetry.addData("kS", kS);
        telemetry.addData("kV", kV);

        telemetry.addData("velocity", shooter.getShooterVelocity());
//        telemetry.addData("ki", kI);
        telemetry.update();
//        if(gamepad1.x) {
        shooter.setCoefficients(kp, kI, kd, kS, kV);
//        }
    }

}