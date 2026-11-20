package org.firstinspires.ftc.teamcode.TeleOpClasses;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Stopper;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.RobotLocalizer;

@TeleOp(name = "Red Right Teleop")
public class RedRightTele extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(telemetry, hardwareMap, RobotLocalizer.allianceColor.BLUE, new Pose2d(68, -68, Math.toRadians(0)), gamepad1, gamepad2);
        robot.start();}

    @Override
    public void loop() {
        robot.update();
    }
}
