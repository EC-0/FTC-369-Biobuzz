package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Posehold {
    private Drivetrain drivetrain;
    private RobotLocalizer robotLocalizer;
    private Pose2d targetPosition;
    private ElapsedTime timer = new ElapsedTime();
    private static double kP_translation, kD_translation;
    private static double kP_rotation, kD_rotation;
    private double prevErrorX, prevErrorY, prevErrorHeading;
    public Posehold(Drivetrain drivetrain, RobotLocalizer robotLocalizer){
        this.drivetrain = drivetrain;
        this.robotLocalizer = robotLocalizer;
    }

    //Going to be used in the future to calculate the dt(derivative) of the change in previous and current errors
    public void lockCurrentPosition(){
        targetPosition = robotLocalizer.getBotPose();
        timer.startTime();
        prevErrorX = 0;
        prevErrorY = 0;
        prevErrorHeading = 0;
    }

    //Takes the change in the derivative of heading, x, and y variables
    public void update(){
        if(targetPosition == null){
            return;
        }
        Pose2d currentPosition = robotLocalizer.getBotPose();
        double dt = timer.seconds();
        timer.reset();
        if(dt == 0){
            dt = 0.01;
        }
        double errorX = targetPosition.position.x - currentPosition.position.x;
        double errorY = targetPosition.position.y - currentPosition.position.y;
        double heading = AngleUnit.normalizeRadians(targetPosition.heading.toDouble() - currentPosition.heading.toDouble());

        double derivativeX = (errorX-prevErrorX) / dt;
        double derivativeY = (errorY-prevErrorY) / dt;
        double derivativeHeading = (heading - prevErrorHeading) / dt;

        prevErrorX = errorX;
        prevErrorY = errorY;
        prevErrorHeading = heading;

        double fieldX = (errorX * kP_translation) + (derivativeX * kD_translation);
        double fieldY = (errorY * kP_translation) + (derivativeY * kD_translation);
        double turn = (heading * kP_rotation) + (heading * kD_rotation);

        double botHeading = currentPosition.heading.toDouble();
        double forward = fieldX * Math.cos(-botHeading) - fieldY * Math.sin(-botHeading);
        double strafe = fieldX * Math.sin(-botHeading) + fieldY * Math.cos(-botHeading);

        double frontLeft = forward + strafe + turn;
        double frontRight = forward - strafe - turn;
        double backLeft = forward - strafe + turn;
        double backRight = forward + strafe - turn;

        drivetrain.drive(frontLeft, frontRight, backLeft, backRight);
    }
}
