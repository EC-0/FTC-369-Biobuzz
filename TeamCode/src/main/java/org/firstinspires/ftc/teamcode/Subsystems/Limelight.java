package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Limelight Test", group = "Vision")
public class Limelight extends OpMode {
    private Limelight3A limelight;
    private static final int PIPELINE = 0;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        telemetry.addLine("limelight initialized");
        telemetry.addData("pipeline", PIPELINE);
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        if(result==null){
            telemetry.addLine("No limelight result");
            telemetry.update();
            return;
        }

        if(result.isValid()){
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();

            telemetry.addData("Result found", "yes");
            telemetry.addData("TX: ", tx);
            telemetry.addData("TY: ",ty);
            telemetry.addData("TA: ", ta);
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        telemetry.addData("April tags", fiducials.size());

        for(LLResultTypes.FiducialResult fiducial : fiducials){
            int id = fiducial.getFiducialId();
            telemetry.addData("AprilTag", "ID: ", id);
        }

        if(result.getBotpose() != null){
            double x = result.getBotpose().getPosition().x;
            double y = result.getBotpose().getPosition().y;
            double z = result.getBotpose().getPosition().z;
            telemetry.addData("Robot Pose", "X: ", "Y: ", "Z: ", x,y,z);
        }

        long staleness = result.getStaleness();
        telemetry.addData("Data age", staleness + "ms");
    }

    @Override
    public void stop(){
        if(limelight != null){
            limelight.stop();
        }
    }
}