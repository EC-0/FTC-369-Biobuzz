package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FlowerIntake {
    Servo flowerIntake;
    public FlowerIntake(HardwareMap hardwareMap){
        flowerIntake = hardwareMap.get(Servo.class, "flowerIntake");
    }

    public void setFlowerIntake(double position){
        flowerIntake.setPosition(position);
    }
}
