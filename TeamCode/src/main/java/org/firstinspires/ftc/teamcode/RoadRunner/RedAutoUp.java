package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class RedAutoUp extends LinearOpMode {

    MecanumDrive drivetrain;
    Pose2d initialPose = new Pose2d(62,12,270);

    @Override
    public void runOpMode() throws InterruptedException {
        TrajectoryActionBuilder startToGarden = drivetrain.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(62, -57), Math.toRadians(270));

        TrajectoryActionBuilder gardenToShoot = startToGarden.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-55,-45), Math.toRadians(270));

        TrajectoryActionBuilder shootToPark = gardenToShoot.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-25,-57), Math.toRadians(270));

        TrajectoryActionBuilder gardenToPark1 = startToGarden.endTrajectory()
                .strafeToLinearHeading(new Vector2d(-25,-40), Math.toRadians(270));

        TrajectoryActionBuilder gardenToPark2 = gardenToPark1.endTrajectory()
                .strafeToLinearHeading(new Vector2d(-25,-57), Math.toRadians(270));

    }

}
