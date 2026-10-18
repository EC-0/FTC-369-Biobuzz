package org.firstinspires.ftc.teamcode.pedro.AutoClasses;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.Paths.RedPaths;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.ivy.Scheduler.execute;
import static com.pedropathing.ivy.Scheduler.schedule;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RedAutoUp extends OpMode {
    private Follower follower;
    private Intake intake;
    private ElapsedTime timer = new ElapsedTime();
    RedPaths pather = new RedPaths();
    Pose2d currentPosition;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(pather.start);
    }

    @Override
    public void start() {
        schedule(RedPaths.redAutoUpRoutine(follower, intake));
    }

    @Override
    public void loop() {
        execute();
    }
}
