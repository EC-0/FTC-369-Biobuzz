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
        fly1.setPower(flyPower);
        fly2.setPower(flyPower);
    }

}
