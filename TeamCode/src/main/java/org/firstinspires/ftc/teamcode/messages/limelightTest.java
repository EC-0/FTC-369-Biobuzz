package org.firstinspires.ftc.teamcode.messages;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;

@TeleOp(name = "limelight test                          ")
public class limelightTest extends OpMode {

    private Limelight limelight;
    @Override
    public void init() {
        limelight= new Limelight(hardwareMap, telemetry);
        limelight.pipelineSwitch(1);
    }

    @Override
    public void loop() {
        limelight.getBotPose();
        limelight.distanceFromTag();
    }
}
