package org.firstinspires.ftc.teamcode.pedro.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbTooManySequentialErrorsException;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.commands.Commands.*;
public class BluePaths {
    public static boolean detectBlueUpTags = true;
    public static boolean detectBlueDownTags = true;

    private static final PoseFactory p = PoseFactory.degrees();

    // Blue Tipped Up
    public static final Pose start = p.of(109, 133, 0);
    public static final Pose garden = p.of(133,133,0);
    public static final Pose flower = p.of(129,94,0);
    public static final Pose shoot = p.of(129,12,0);
    public static final Pose park = p.of(129,47,0);

    // Best case scenario only 3 paths
    public static Path goToGarden() {
        return line(start, garden).linear(start, garden);
    }
    public static Path gardenToShoot() {
        return curve(garden,new Pose(105,50), shoot).linear(garden, shoot);
    }
    public static Path shootToPark() {
        return line(shoot, park).linear(shoot, park);
    }
    // all the backup paths
    public static Path gardenToFlower() {
        return curve(garden, new Pose(115,110), flower).linear(garden, flower);
    }
    public static Path flowerToStart() {
        return curve(flower, new Pose(115,110), start).linear(flower, start);
    }

    public static Path flowerToShoot() {
        return curve(flower, new Pose(100,61), shoot).linear(flower, shoot);
    }
    public static Path shootToFlower() {
        return curve(shoot, new Pose(100, 61), flower).linear(shoot, flower);
    }
    public static Path startToPark() {
        return line(start, park).linear(start, park);
    }
    public static Path shootToStart() {
        return line(shoot, start).linear(shoot, start);
    }
    public static Path startToFlower() {
        return curve(start, new Pose(115,110), flower).linear(start, flower);
    }
    public static Path placeholder() {
        return line(new Pose(1,1), new Pose(2,2));
    }

    public static Command blueAutoUpRoutine(Follower follower, Intake intake) {
        return sequential(
                // shoot
                waitMs(5000),
                conditional(
                        () -> detectBlueUpTags,
                        sequential(
                                intake.autoIntake(0.8),
                                follow(follower, BluePaths.goToGarden()),
                                intake.autoIntake(0.0),
                                // shoot
                                conditional(
                                        () -> detectBlueUpTags,
                                        sequential(
                                                intake.autoIntake(0.8),
                                                follow(follower, BluePaths.gardenToFlower()),
                                                waitMs(5000),
                                                intake.autoIntake(0.8),
                                                follow(follower, BluePaths.flowerToStart()),
                                                // shoot
                                                follow(follower, BluePaths.startToPark())
                                        ),
                                        sequential(
                                                intake.autoIntake(0.8),
                                                follow(follower, BluePaths.gardenToFlower()),
                                                waitMs(5000),
                                                intake.autoIntake(0.8),
                                                follow(follower, BluePaths.flowerToShoot()),
                                                waitMs(5000),
                                                conditional(
                                                        () -> detectBlueDownTags,
                                                        sequential(
                                                                // shoot
                                                                follow(follower, BluePaths.shootToPark())
                                                        ),
                                                        follow(follower, BluePaths.shootToPark())
                                                )
                                        )
                                )
                        ),
                        sequential(
                                intake.autoIntake(0.8),
                                follow(follower, BluePaths.goToGarden()),
                                intake.autoIntake(0.0),
                                follow(follower, BluePaths.gardenToShoot()),
                                conditional(
                                        () -> detectBlueDownTags,
                                        sequential(
                                                // shoot
                                                waitMs(5000),
                                                conditional(
                                                        () -> detectBlueDownTags,
                                                        sequential(
                                                                intake.autoIntake(0.8),
                                                                follow(follower, BluePaths.shootToFlower()),
                                                                waitMs(5000),
                                                                intake.autoIntake(0.0),
                                                                follow(follower, BluePaths.flowerToShoot()),
                                                                // shoot
                                                                follow(follower, BluePaths.shootToPark())
                                                        ),
                                                        follow(follower, BluePaths.shootToPark())
                                                )
                                        ),
                                        follow(follower, BluePaths.shootToPark())
                                )
                        )
                )
        );
    }
    public static final Pose start2 = p.of(133,8,270);
    public static final Pose flower2 = p.of(94,12,270);
    public static final Pose park2 = p.of(133,25,180);
    public static final Pose shoot2 = p.of(106,106,270);
    public static Path startToFlower2() {
        return curve(start2, new Pose(100,20), flower2).linear(start2, flower2);
    }
    public static Path flowerToPark2() {
        return curve(flower2, new Pose(100,25), park2).linear(flower2, park2);
    }
    public static Path flowerToShoot2() {
        return curve(flower2, new Pose(120,50), shoot2).linear(flower2, shoot2);
    }
    public static Path shootToFlower2() {
        return curve(shoot2, new Pose(120,50), flower2).linear(shoot2, flower2);
    }
    public static Path shootToPark2() {
        return line(shoot2, park2).linear(shoot2, park2);
    }
    public static Command blueAutoDownRoutine(Follower follower, Intake intake) {
        return sequential(
                follow(follower, BluePaths.startToFlower2()),
                waitMs(5000),
                conditional(
                        () -> !detectBlueDownTags,
                        sequential(
                                follow(follower, BluePaths.flowerToShoot2()),
                                conditional(
                                        () -> detectBlueUpTags,
                                        sequential(
                                                // shoot
                                                waitMs(5000),
                                                conditional(
                                                        () -> detectBlueUpTags,
                                                        sequential(
                                                                follow(follower, BluePaths.shootToFlower2()),
                                                                conditional(
                                                                        () -> detectBlueDownTags,
                                                                        sequential(
                                                                                intake.autoIntake(0.8),
                                                                                waitMs(5000),
                                                                                intake.autoIntake(0.0),
                                                                                // shoot
                                                                                follow(follower, BluePaths.flowerToPark2())
                                                                        ),
                                                                        sequential(
                                                                                follow(follower, BluePaths.flowerToShoot2()),
                                                                                conditional(
                                                                                        () -> detectBlueUpTags,
                                                                                        sequential(
                                                                                                // shoot
                                                                                                follow(follower, BluePaths.shootToPark2())
                                                                                        ),
                                                                                        sequential(
                                                                                                follow(follower, BluePaths.shootToFlower2()),
                                                                                                // shoot
                                                                                                follow(follower, BluePaths.flowerToPark2())
                                                                                        )
                                                                                )
                                                                        )
                                                                )

                                                        ),
                                                        sequential(
                                                                follow(follower, BluePaths.shootToFlower2()),
                                                                // shoot
                                                                intake.autoIntake(0.8),
                                                                waitMs(5000),
                                                                intake.autoIntake(0.8),
                                                                // shoot
                                                                follow(follower, BluePaths.flowerToPark2())
                                                        )
                                                )

                                        ),
                                        sequential(
                                                follow(follower, BluePaths.shootToFlower2()),
                                                // shoot
                                                intake.autoIntake(0.8),
                                                waitMs(5000),
                                                intake.autoIntake(0.0),
                                                // shoot
                                                follow(follower, BluePaths.flowerToPark2())
                                        )
                                )
                        ),
                        sequential(
                                // shoot
                                intake.autoIntake(0.8),
                                waitMs(5000),
                                intake.autoIntake(0.0),
                                // shoot
                                follow(follower, BluePaths.flowerToPark2())
                        )
                )
        );
    }
}