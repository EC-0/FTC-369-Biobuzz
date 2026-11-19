package org.firstinspires.ftc.teamcode.RoadRunner;

import android.app.slice.SliceMetrics;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.RobotLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class BackgroundActions {
    Turret turret;
    RobotLocalizer localizer;
    Limelight limelight;
    Shooter shooter;

    public BackgroundActions(Turret turret, RobotLocalizer localizer, Limelight limelight, Shooter shooter) {
        this.turret = turret;
        this.localizer = localizer;
        this.limelight = limelight;
        this.shooter = shooter;
    }

    // the aiming part of the turret
    public Action update() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                turret.update(localizer.angleToGoal());
                return true;
            }
        };
    }

    // sets the power of the flywheel
    public Action setTurretPower() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shooter.regression(localizer.getDistanceToGoal());
                return true;
            }
        };
    }

    // updates the localizer
    public Action updateLocalizer() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                localizer.updateBotPosition();
                return true;
            }
        };
    }

    // updates and gets the closest april tag
    public Action getClosest() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                limelight.getClosest();
                return true;
            }
        };
    }

    // combines update(), setTurretPower() and getClosest()
    public Action combinedActions() {
        return  new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                localizer.updateBotPosition();
                limelight.getClosest();
                shooter.regression(localizer.getDistanceToGoal());
                turret.update(localizer.angleToGoal());
                return true;
            }
        };
    }

    // only returns and ends the action when it spots tags
    public Action getTag() {
        return  new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return (limelight.getTag() == null);
            }
        };
    }

    // only returns and ends the action when it doesn't spot a tag
     public Action dontGetTag() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return (limelight.getTag() != null);
            }
        };
     }
}
