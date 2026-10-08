package org.firstinspires.ftc.teamcode.TeleOpClasses;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;


@TeleOp (name = "Test TeleOp")
public class TestTeleop extends OpMode {

    Limelight limelight;
    Drivetrain drivetrain;
    Intake intake;
    Turret turret;

    @Override
    public void init() {

        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap, telemetry);
        limelight.pipelineSwitch(0);
    }

    @Override
    public void loop() {
        drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.left_trigger,
                gamepad1.right_trigger);

        if (gamepad1.a) {
            /*if (intake.getPower() > 0.0) {
                intake.setPower(0.8);
            }
            else {
                intake.setPower(0.0);
            }
            */
             intake.intakePower(0.8);
        }
        if(gamepad1.bWasPressed()){
            turret.setMotorManual(1700);
            turret.manualFlyMode();
        }

        /*
        else if(gamepad1.bWasPressed()){
            turret.regression(limelight.distanceFromTag());
        }
        */

    }
}
