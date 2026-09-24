package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Limelight {
    private Limelight3A limelight;
    private static int PIPELINE = 0;
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 49.021;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 13.0;

    // distance from the target to the floor
    double goalHeightInches = 65.6;

    private Telemetry telemetry;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        telemetry.addLine("limelight initialized");
        telemetry.addData("pipeline", PIPELINE);
        telemetry.update();
        this.telemetry = telemetry;
    }

    public void pipelineSwitch(int PIPELINE) {
        limelight.pipelineSwitch(PIPELINE);
    }

    public void distanceFromTag(){
        LLResult result = getResult();
        if(result==null){
            return;
        }
        double tx = result.getTx();
        double ty = result.getTy();
        double ta = result.getTa();

        double angleToGoalDegrees = limelightMountAngleDegrees + ty;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    private LLResult getResult(){
        LLResult result = limelight.getLatestResult();
        if(result!=null && result.isValid()){
            return result;
        }
        else{
            telemetry.addLine("No limelight result");
            telemetry.update();
            return null;
        }
    }

    public void getBotPose() {
        LLResult result = getResult();
        if(result==null){
            return;
        }
        double tx = result.getTx();
        double ty = result.getTy();
        double ta = result.getTa();

        telemetry.addData("Result found", "yes");
        telemetry.addData("TX: ", tx);
        telemetry.addData("TY: ",ty);
        telemetry.addData("TA: ", ta);


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



    public void stop(){
        if(limelight != null){
            limelight.stop();
        }
    }
}