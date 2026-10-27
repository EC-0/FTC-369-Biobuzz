package org.firstinspires.ftc.teamcode.pedro.AutoClasses;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
@Autonomous
public class TestAuto extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose pose1 = p.of(10, 10, 90);
    private final Pose pose2 = p.of(10, 50, 90);
    private final Pose pose3 = p.of(30, 10, 90);
    private Path line1() {
        return line(pose1, pose2).linear(pose1, pose2);
    }
    private Path line2() { return line(pose2, pose3).linear(pose2, pose3); }
    private Path line3() { return  line(pose3, pose1).linear(pose3, pose1); }

    private Command autoRoutine() {
        return sequential(
                follow(follower, line1()),
                follow(follower, line2()),
                follow(follower, line3())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(pose1);
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }
}
