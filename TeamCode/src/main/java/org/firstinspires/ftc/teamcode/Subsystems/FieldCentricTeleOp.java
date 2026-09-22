package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
// import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
// import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name = "Field Centric", group = "TeleOp")
public class FieldCentricTeleOp extends OpMode {
    Drivetrain drivetrain;
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
    }

    @Override
    public void loop() {
        drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.left_bumper
                ,gamepad1.right_bumper);

        if (gamepad1.aWasPressed()) {
            drivetrain.resetHeading();
        }
    }
}
