package org.firstinspires.ftc.teamcode.pedro.AutoClasses;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
@Autonomous
public class RedAuto extends OpMode {

    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    // these methods are for when you're on the side that's tipped up
    private final Pose startPos = p.of(59, 8, 90);
    private final Pose shootPos = p.of(59, 23, 90);
    private final Pose lowerPos = p.of(30,9,180);
    private final Pose ballPos = p.of(8,9,180);
    private final Pose otherShootPos = p.of(59,110,270);
    private Path goToShoot() {
        return line(startPos, shootPos).linear(startPos, shootPos);
    }
    private Path getReady() {
        return line(shootPos, lowerPos).linear(shootPos, lowerPos);
    }
    private Path getBalls() {
        return line(lowerPos, ballPos).linear(lowerPos, ballPos);
    }
    private Path goToOtherShoot() {
        return curve(ballPos, new Pose(32, 119), otherShootPos).linear(ballPos, otherShootPos);
    }

    private Command redAutoUp() {
        return sequential(
                follow(follower, goToShoot()),
                // shoots balls
                follow(follower, getReady()),
                // turns on intake
                follow(follower, getBalls()),
                // waits a bit for all the balls to intake
                // turns off intake
                follow(follower, goToOtherShoot())
                // shoot again
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPos);
    }

    @Override
    public void start() {
        schedule(redAutoUp());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }
}
