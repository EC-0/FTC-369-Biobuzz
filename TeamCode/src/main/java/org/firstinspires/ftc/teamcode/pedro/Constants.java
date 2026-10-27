package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.9864559323768916);
        c.yPodOffset.set(1.8691683191014088);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2809035834567396);
                Controller secondaryTranslationalForward = Controller.proportional(0.10378636820527751);
                Controller primaryTranslationalLateral = Controller.proportional(0.35341403710829766);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1305770433145229);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016545926888882675));
                c.brake.set(Controller.proportionalFeedforward(0.014064037855550273));

                c.headingFeedback.set(Controller.proportional(4.310002187143355));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06044848016608528, 0.006558997953384004));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07227460249931252, 0.06528572325936366));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0019298616013995575, 0.00191201629022016));

                c.maxAchievableForwardVelocity.set(62.86738586729156);
                c.maxAchievableStrafeVelocity.set(52.566658000385566);
                c.naturalForwardDeceleration.set(32.44428963426015);
                c.naturalStrafeDeceleration.set(62.85123246041106);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
             new PinpointLocalizer(h,localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}