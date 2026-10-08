package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret{
    DcMotorEx fly1;
    DcMotorEx fly2;
    double flyPower;
    boolean useRegression;
    public Turret(HardwareMap hardwareMap) {
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        flyPower = 0;
    }

    public void setMotorManual(double increment){
        flyPower += increment;
    }

    public void manualFlyMode() {
        fly1.setVelocity(flyPower);
        fly2.setVelocity(flyPower);
    }


    //Regression method depending on distance to hive
    public void regression(double distanceToHive){
        double targetVelocity = distanceToHive; //regression variable
        fly1.setVelocity(targetVelocity);
        fly2.setVelocity(targetVelocity);
    }


}
