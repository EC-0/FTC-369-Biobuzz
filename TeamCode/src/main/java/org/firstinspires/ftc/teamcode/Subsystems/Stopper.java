package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Stopper {
    Servo stopper;
    Stopper(HardwareMap hardwareMap){
        stopper = hardwareMap.get(Servo.class, "stopper");
    }

    public void setStopper(double stopperPos){
        stopper.setPosition(stopperPos);
    }
}
