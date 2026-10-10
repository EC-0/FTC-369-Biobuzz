package org.firstinspires.ftc.teamcode.pedro.AutoClasses;
import org.firstinspires.ftc.teamcode.pedro.Paths.RedPaths;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.ivy.Scheduler.execute;
import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RedAutoUp extends OpMode {
    private Follower follower;
    RedPaths pather = new RedPaths();

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(pather.start);
    }

    @Override
    public void start() {
        schedule(RedPaths.redAutoUpRoutine(follower));
    }

    @Override
    public void loop() {
        execute();
    }
}
