package org.firstinspires.ftc.teamcode.TeleOpClasses;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Limelight TeleOp Test")
public class limelightTest extends OpMode {
    Limelight limelighter;
    Drivetrain drivetrain;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        limelighter = new Limelight(hardwareMap);
        limelighter.switchPipeline(0);
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            drivetrain.resetHeading();
        }

        drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.left_trigger,
                gamepad1.right_trigger);

        limelighter.getClosest();
        telemetry.addData("Closest Tag", limelighter.getTag());
        telemetry.addData("Distance to tag", limelighter.getDistance());
        telemetry.update();
    }
}
