package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

// finished drive train auto V1

@Autonomous(name = "autoTest")
public class autoTest extends LinearOpMode {
    MecanumDrive driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, 10, Math.toRadians(315));

        TrajectoryActionBuilder path1 = driveTrain.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(50, 10), Math.toRadians(90));

        TrajectoryActionBuilder path2 = path1.endTrajectory().fresh()
                .strafeTo(new Vector2d(50, 10));

        TrajectoryActionBuilder path3 = path2.endTrajectory().fresh()
                .strafeTo(new Vector2d(10,10));

        while (!opModeIsActive()) {
            if (isStopRequested()) {
                driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                return;
            }
        }

        SequentialAction Perform = new SequentialAction(
                path1.build(),
                path2.build(),
                path3.build()
        );

        Actions.runBlocking(Perform);

    }
}
