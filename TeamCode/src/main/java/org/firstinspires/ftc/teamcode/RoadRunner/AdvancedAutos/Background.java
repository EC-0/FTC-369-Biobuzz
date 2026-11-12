package org.firstinspires.ftc.teamcode.RoadRunner.AdvancedAutos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.RaceAction;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

public class Background {

    static Turret turret;
    static Intake intake;

    public Background(Turret turret, Intake intake) {
        this.turret = turret;
        this.intake = intake;
    }


    public static Action runWithTurret(Action path, Action flywheeler, Action aim) {
        return new RaceAction(path, flywheeler, aim);
    }

    public static Action runFlywheel(double distance) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                turret.setTurretPower(distance);
                return false;
            }
        };
    }

    public static Action aim(double botErrorDeg) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                turret.update(botErrorDeg);
                return false;
            }
        };
    }
}
