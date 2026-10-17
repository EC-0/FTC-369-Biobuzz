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
public class RedPaths {
    public static boolean detectRedUpTags = true;
    public static boolean detectRedDownTags = true;

    private static final PoseFactory p = PoseFactory.degrees();

    // Red Tipped Up
    public static final Pose start = p.of(35, 8, 180);
    public static final Pose garden = p.of(8,8,180);
    public static final Pose flower = p.of(11,47,180);
    public static final Pose shoot = p.of(11,130,180);
    public static final Pose park = p.of(9,95,180);

    // Best case scenario only 3 paths
    public static Path goToGarden() {
        return line(start, garden).linear(start, garden);
    }
    public static Path gardenToShoot() {
        return curve(garden,new Pose(30,61), shoot).linear(garden, shoot);
    }
    public static Path shootToPark() {
        return line(shoot, park).linear(shoot, park);
    }
    // all the backup paths
    public static Path gardenToFlower() {
        return curve(garden, new Pose(25,25), flower).linear(garden, flower);
    }
    public static Path flowerToStart() {
        return curve(flower, new Pose(25,25), start).linear(flower, start);
    }

    public static Path flowerToShoot() {
        return curve(flower, new Pose(30,61), shoot).linear(flower, shoot);
    }
    public static Path shootToFlower() {
        return curve(shoot, new Pose(61, 30), flower).linear(shoot, flower);
    }
    public static Path startToPark() {
        return line(start, park).linear(start, park);
    }
    public static Path shootToStart() {
        return line(shoot, start).linear(shoot, start);
    }
    public static Path startToFlower() {
        return curve(start, new Pose(30,50), flower).linear(start, flower);
    }
    public static Path placeholder() {
        return line(new Pose(1,1), new Pose(2,2));
    }

    public static Command redAutoUpRoutine(Follower follower, Intake intake) {
        return sequential(
                // shoot
                waitMs(5000),
                conditional(
                        () -> detectRedUpTags,
                        sequential(
                                intake.autoIntake(0.8),
                                follow(follower, RedPaths.goToGarden()),
                                intake.autoIntake(0.0),
                                // shoot
                                conditional(
                                        () -> detectRedUpTags,
                                        sequential(
                                                intake.autoIntake(0.8),
                                                follow(follower, RedPaths.gardenToFlower()),
                                                waitMs(5000),
                                                intake.autoIntake(0.8),
                                                follow(follower, RedPaths.flowerToStart()),
                                                // shoot
                                                follow(follower, RedPaths.startToPark())
                                        ),
                                        sequential(
                                                intake.autoIntake(0.8),
                                                follow(follower, RedPaths.gardenToFlower()),
                                                waitMs(5000),
                                                intake.autoIntake(0.8),
                                                follow(follower, RedPaths.flowerToShoot()),
                                                waitMs(5000),
                                                conditional(
                                                        () -> detectRedDownTags,
                                                        sequential(
                                                                // shoot
                                                                follow(follower, RedPaths.shootToPark())
                                                        ),
                                                        follow(follower, RedPaths.shootToPark())
                                                )
                                        )
                                )
                        ),
                        sequential(
                                intake.autoIntake(0.8),
                                follow(follower, RedPaths.goToGarden()),
                                intake.autoIntake(0.0),
                                follow(follower, RedPaths.gardenToShoot()),
                                conditional(
                                        () -> detectRedDownTags,
                                        sequential(
                                                // shoot
                                                waitMs(5000),
                                                conditional(
                                                        () -> detectRedDownTags,
                                                        sequential(
                                                                intake.autoIntake(0.8),
                                                                follow(follower, RedPaths.shootToFlower()),
                                                                waitMs(5000),
                                                                intake.autoIntake(0.0),
                                                                follow(follower, RedPaths.flowerToShoot()),
                                                                // shoot
                                                                follow(follower, RedPaths.shootToPark())
                                                        ),
                                                        follow(follower, RedPaths.shootToPark())
                                                )
                                        ),
                                        follow(follower, RedPaths.shootToPark())
                                )
                        )
                )
        );
    }
    public static final Pose start2 = p.of(8,133,90);
    public static final Pose flower2 = p.of(27,110,90);
    public static final Pose park2 = p.of(9,120,180);
    public static final Pose shoot2 = p.of(35,35,90);
    public static Path startToFlower2() {
        return curve(start2, new Pose(27,110), flower2).linear(start2, flower2);
    }
    public static Path flowerToPark2() {
        return curve(flower2, new Pose(27,110), park2).linear(flower2, park2);
    }
    public static Path flowerToShoot2() {
        return curve(flower2, new Pose(20,50), shoot2).linear(flower2, shoot2);
    }
    public static Path shootToFlower2() {
        return curve(shoot2, new Pose(20,50), flower2).linear(shoot2, flower2);
    }
    public static Path shootToPark2() {
        return line(shoot2, park2).linear(shoot2, park2);
    }
    public static Command redAutoDownRoutine(Follower follower, Intake intake) {
        return sequential(
                follow(follower, RedPaths.startToFlower2()),
                waitMs(5000),
                conditional(
                        () -> !detectRedDownTags,
                        sequential(
                                follow(follower, RedPaths.flowerToShoot2()),
                                conditional(
                                        () -> detectRedUpTags,
                                        sequential(
                                                // shoot
                                                waitMs(5000),
                                                conditional(
                                                        () -> detectRedUpTags,
                                                        sequential(
                                                                follow(follower, RedPaths.shootToFlower2()),
                                                                conditional(
                                                                        () -> detectRedDownTags,
                                                                        sequential(
                                                                                intake.autoIntake(0.8),
                                                                                waitMs(5000),
                                                                                intake.autoIntake(0.0),
                                                                                // shoot
                                                                                follow(follower, RedPaths.flowerToPark2())
                                                                        ),
                                                                        sequential(
                                                                                follow(follower, RedPaths.flowerToShoot2()),
                                                                                conditional(
                                                                                        () -> detectRedUpTags,
                                                                                        sequential(
                                                                                                // shoot
                                                                                                follow(follower, RedPaths.shootToPark2())
                                                                                        ),
                                                                                        sequential(
                                                                                                follow(follower, RedPaths.shootToFlower2()),
                                                                                                // shoot
                                                                                                follow(follower, RedPaths.flowerToPark2())
                                                                                        )
                                                                                )
                                                                        )
                                                                )

                                                        ),
                                                        sequential(
                                                                follow(follower, RedPaths.shootToFlower2()),
                                                                // shoot
                                                                intake.autoIntake(0.8),
                                                                waitMs(5000),
                                                                intake.autoIntake(0.8),
                                                                // shoot
                                                                follow(follower, RedPaths.flowerToPark2())
                                                        )
                                                )

                                        ),
                                        sequential(
                                                follow(follower, RedPaths.shootToFlower2()),
                                                // shoot
                                                intake.autoIntake(0.8),
                                                waitMs(5000),
                                                intake.autoIntake(0.0),
                                                // shoot
                                                follow(follower, RedPaths.flowerToPark2())
                                        )
                                )
                        ),
                        sequential(
                                // shoot
                                intake.autoIntake(0.8),
                                waitMs(5000),
                                intake.autoIntake(0.0),
                                // shoot
                                follow(follower, RedPaths.flowerToPark2())
                        )
                )
        );
    }
}