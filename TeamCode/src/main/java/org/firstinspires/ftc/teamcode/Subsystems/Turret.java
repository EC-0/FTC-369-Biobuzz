package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Turret{


    public DcMotorEx fly1;
    public DcMotorEx fly2;
    public DcMotorEx turret;

    private double m_robotRelativeAngle;
    private double m_fieldRelativeAngle;
    public double flyPower;
    private double offsetX = 0.0;
    private double offsetY = 0.0;

    private final double totalTurnTicks = 853.0;
    private final double totalDegrees = 320.0;
    private final double ticksPerDegree = totalTurnTicks / totalDegrees;


    boolean useRegression;
    public Turret(HardwareMap hardwareMap) {
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        flyPower = 0;
        fly1.setDirection(DcMotorSimple.Direction.REVERSE);
        fly1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void setTurretPower(double velocity){
        fly1.setVelocity(velocity);
        fly2.setVelocity(velocity);
    }

    public void turretTrackPosition(Pose2d target, Pose2d robotPose){
        Pose2d turretOffset = new Pose2d(new Vector2d(offsetX, offsetY), Math.toDegrees(0));
        Pose2d turretPose = robotPose.times(turretOffset);
        double dY = target.position.y - turretPose.position.y;
        double dX = target.position.x - turretPose.position.x;

        double fieldAngleRadians = Math.atan2(dY, dX);
        double robotHeadingRadians = robotPose.heading.log();

        double robotRelativeAngleRadians = fieldAngleRadians - robotHeadingRadians;
        m_fieldRelativeAngle = Math.toDegrees(fieldAngleRadians);
        m_robotRelativeAngle = Math.toDegrees(robotRelativeAngleRadians);

        this.setPivotPosition(m_robotRelativeAngle);


        /*
        Rotation2d fieldRelativeAngle = Rotation2d.exp(Math.atan2(dY, dX));
        Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(robotPose.heading);
        m_robotRelativeAngle = Math.toDegrees(robotRelativeAngle.log());
        m_fieldRelativeAngle = Math.toDegrees(fieldRelativeAngle.log());
        */

    }

    private void setPivotPosition(double degrees){
        double clampedDegrees = Math.max(-150.0, Math.min(150.0, degrees));
        int targetTicks = (int)(clampedDegrees * ticksPerDegree);
        turret.setTargetPosition(targetTicks);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPower(0.8);
    }




    public void regression(double distanceToHive){
        double targetVelocity = distanceToHive; //regression variable
        fly1.setVelocity(targetVelocity);
        fly2.setVelocity(targetVelocity);
    }




}
