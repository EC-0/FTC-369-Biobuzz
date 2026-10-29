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
    Limelight3A limelighter;
    double targetHeight = 65.6, cameraHeight = 13.0, cameraAngle = 49.021;
    double deltaHeight = targetHeight - cameraHeight;
    double pitchAngle;
    Integer bestId = null;
    Double lowestDistance = null;
    public Limelight(HardwareMap hardwareMap) {

        limelighter = hardwareMap.get(Limelight3A.class, "limelight");
        limelighter.pipelineSwitch(0);
        limelighter.setPollRateHz(100);
        limelighter.start();

    }

    public void getClosest() {

        bestId = null;
        lowestDistance = null;

        LLResult result = limelighter.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tagList = result.getFiducialResults();

            if (!tagList.isEmpty()) {

                for (LLResultTypes.FiducialResult tag : tagList) {
                    if (lowestDistance != null) {

                        pitchAngle = cameraAngle + tag.getTargetYDegrees();

                        if ((returnDistance(deltaHeight, pitchAngle)) < lowestDistance && returnDistance(deltaHeight,pitchAngle) < 120) {
                            bestId = tag.getFiducialId();
                            lowestDistance = returnDistance(deltaHeight, pitchAngle);
                        }
                    }
                    else {
                        pitchAngle = cameraAngle + tag.getTargetYDegrees();

                        bestId = tag.getFiducialId();
                        lowestDistance = returnDistance(deltaHeight, pitchAngle);
                    }
                }
            }
            else {
                lowestDistance = null;
                bestId = null;
            }
        }
    }

    public Integer getTag() {
        return bestId;
    }

    public Double getDistance() {
        return lowestDistance;
    }

    public static double returnDistance(double dh, double angle) {
        return dh / Math.tan(Math.toRadians(angle));
    }

    public void switchPipeline(int pipeline) {
        limelighter.pipelineSwitch(pipeline);
    }
}
