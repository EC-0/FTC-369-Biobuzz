package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

public class OdomLimelightTurret {

    public enum Mode {
        ODOMETRY_AUTO_MODE,
        LIMELIGHT_ASSIST_MODE,
        LIMELIGHT_BASIC_MODE,
        MANUAL_RESET_MODE,
        LOCKED_TURRET_MODE
    }

    private LLResult llResult;
    private Mode turretMode = Mode.ODOMETRY_AUTO_MODE;
    private double botErrorDeg = 0.0;
    private boolean llHasTarget = false;
    private double llTxDeg = 0.0;
    private double llLastTxDeg = 0.0;
    private long llLastSeenNanos = 0;
    private double llGracePeriodSec = 0.25;
    private double llFallbackToOdoSec = 0.8;

    private int minAllowedTicks = -833;
    private int maxAllowedTicks = -20;

    private boolean manualResetHeld = false;
    private double manualResetPower = 0.7;
    private int manualResetDirection = 1;
    private boolean wasManualResetHeld = false;

    private boolean holdTurretForPoseHold = false;

    private double ll_kP = 0.01;
    private double ll_kD = 0.000;
    private double ll_kS = 0.04;
    private double ll_maxOutput = 0.8;
    private double ll_deadbandDeg = 0.0;

    private double ll_prevErr = 0.0;
    private long ll_prevTimeNanos = 0;

    private double manualControl = 0.0;

    public HardwareMap hardwareMap;
    public Turret turret;
    public Limelight limelight;

    public OdomLimelightTurret(HardwareMap hardwareMap, RobotLocalizer.allianceColor allianceColor) {
        this.hardwareMap = hardwareMap;
        this.turret = new Turret(hardwareMap);
        this.limelight = new Limelight(hardwareMap);

        if (allianceColor == RobotLocalizer.allianceColor.RED) {
            limelight.switchPipeline(1);
        } else {
            limelight.switchPipeline(0);
        }
    }

    public void setTurretMode(Mode turretMode) {
        if (turretMode == Mode.LOCKED_TURRET_MODE) {
            resetLimelightPIDState();
            llLastTxDeg = 0.0;
            llLastSeenNanos = 0;
        }
        this.turretMode = turretMode;
    }

    public void setManualResetHeld(boolean held) {
        this.manualResetHeld = held;
    }

    public void setManualControl(double manualControl) {
        this.manualControl = manualControl;
    }

    public void setBotErrorDeg(double botErrorDeg) {
        this.botErrorDeg = botErrorDeg;
    }

    public void setLlHasTarget(boolean hasTarget) {
        this.llHasTarget = hasTarget;
    }

    public void setHoldTurretForPoseHold(boolean hold) {
        this.holdTurretForPoseHold = hold;
    }

    public Mode getTurretMode() {
        return turretMode;
    }

    public void resetPosition() {
        turret.resetPosition();
    }

    public void update() {
        switch (turretMode) {
            case MANUAL_RESET_MODE: {
                if (manualResetHeld) {
                    wasManualResetHeld = true;
                    turret.setPowerRaw(manualResetPower * manualResetDirection);
                } else {
                    if (wasManualResetHeld) {
                        turret.setPowerRaw(0.0);
                        turret.resetPosition();
                        wasManualResetHeld = false;
                        setTurretMode(Mode.LIMELIGHT_BASIC_MODE);
                    } else {
                        turret.setPowerRaw(0.0);
                    }
                }
                break;
            }

            case LIMELIGHT_ASSIST_MODE: {
                if (holdTurretForPoseHold) {
                    turret.setPowerRaw(0.0);
                    break;
                }

                if (Math.abs(manualControl) > 0) {
                    turret.manual(manualControl);
                    resetLimelightPIDState();
                } else if (!llHasTarget) {
                    turret.update(botErrorDeg);
                    turret.aimPIDF();
                    resetLimelightPIDState();
                } else {
                    runLimelightAiming();
                }
                break;
            }

            case LIMELIGHT_BASIC_MODE: {
                if (holdTurretForPoseHold) {
                    turret.setPowerRaw(0.0);
                    break;
                }

                if (Math.abs(manualControl) > 0) {
                    turret.manual(manualControl);
                    resetLimelightPIDState();
                } else {
                    runLimelightAiming();
                }
                break;
            }

            case ODOMETRY_AUTO_MODE: {
                if (llHasTarget) {
                    turret.update(botErrorDeg);
                    turret.aimPIDF();
                } else {
                    turret.setPowerRaw(0.0);
                }
                break;
            }

            case LOCKED_TURRET_MODE: {
                turret.setPowerRaw(0.0);
                break;
            }
        }
    }

    private void runLimelightAiming() {
        llResult = limelight.limelighter.getLatestResult();
        boolean hasTargetNow = false;
        double txNow = 0.0;

        if (llResult != null && llResult.isValid()) {
            hasTargetNow = true;
            txNow = llResult.getTx();
            llTxDeg = txNow;
        }

        if (hasTargetNow) {
            double error = -txNow;
            long now = System.nanoTime();
            double derivative = 0.0;

            if (ll_prevTimeNanos != 0) {
                double dt = (now - ll_prevTimeNanos) / 1e9;
                if (dt > 1e-6) {
                    derivative = (error - ll_prevErr) / dt;
                }
            }

            ll_prevErr = error;
            ll_prevTimeNanos = now;

            double output = (ll_kP * error) + (ll_kD * derivative);

            if (Math.abs(error) <= ll_deadbandDeg) {
                output = 0.0;
            } else {
                if (error > 0) {
                    output += Math.abs(ll_kS);
                } else if (error < 0) {
                    output -= Math.abs(ll_kS);
                }
            }

            if (output > ll_maxOutput) output = ll_maxOutput;
            if (output < -ll_maxOutput) output = -ll_maxOutput;

            double currentTicks = turret.getCurrentTicks();
            if (currentTicks <= minAllowedTicks && output < 0) {
                output = 0.0;
            }
            if (currentTicks >= maxAllowedTicks && output > 0) {
                output = 0.0;
            }

            //Now accurately passes the calculated PID output to the motor
            turret.setPowerRaw(output);
        } else {
            turret.setPowerRaw(0.0);
            llTxDeg = 0.0;
            resetLimelightPIDState();
        }
    }

    private void resetLimelightPIDState() {
        ll_prevErr = 0.0;
        ll_prevTimeNanos = 0;
    }
}
