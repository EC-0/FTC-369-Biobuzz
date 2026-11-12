package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Limelight {
    public Limelight3A limelighter;
    double targetHeight = 65.6, cameraHeight = 11.6929, cameraAngle = 49.021;
    double deltaHeight = targetHeight - cameraHeight;
    double pitchAngle;

    Integer bestId = null;
    Double lowestDistance = null;
    double bestTx = 0.0;

    public Limelight(HardwareMap hardwareMap) {
        limelighter = hardwareMap.get(Limelight3A.class, "limelight");
        limelighter.pipelineSwitch(0);
        limelighter.setPollRateHz(100);
        limelighter.start();
    }

    public void getClosest() {
        bestId = null;
        lowestDistance = null;
        bestTx = 0.0;

        LLResult result = limelighter.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tagList = result.getFiducialResults();



            if (tagList != null && !tagList.isEmpty()) {


                for (LLResultTypes.FiducialResult tag : tagList) {

                    pitchAngle = cameraAngle + tag.getTargetYDegrees();
                    double dist = returnDistance(deltaHeight, pitchAngle);

                    if (tag.getFiducialId() == 43 || tag.getFiducialId() == 44 || tag.getFiducialId() == 39 || tag.getFiducialId() == 40) {

                        if (bestId != 43 || bestId != 44 || bestId != 39 || bestId != 40) {
                            bestId = tag.getFiducialId();
                            bestTx = tag.getTargetXDegrees();
                            lowestDistance = dist;
                        }
                        else {
                            if (dist < lowestDistance) {
                                bestId = tag.getFiducialId();
                                bestTx = tag.getTargetXDegrees();
                                lowestDistance = dist;
                            }
                        }
                    }
                    else {
                        if (lowestDistance == null) {
                            bestId = tag.getFiducialId();
                            lowestDistance = dist;
                            bestTx = tag.getTargetXDegrees();
                        } else if (dist < lowestDistance && dist < 120) {
                            bestId = tag.getFiducialId();
                            lowestDistance = dist;
                            bestTx = tag.getTargetXDegrees();
                        }
                    }
                }
            }
        }
    }

    public Integer getTag() {
        return bestId;
    }

    public Double getDistance() {
        return lowestDistance;
    }

    public double getTx() {
        return bestTx;
    }

    public static double returnDistance(double dh, double angle) {
        return dh / Math.tan(Math.toRadians(angle));
    }

    public void switchPipeline(int pipeline) {
        limelighter.pipelineSwitch(pipeline);
    }
}
