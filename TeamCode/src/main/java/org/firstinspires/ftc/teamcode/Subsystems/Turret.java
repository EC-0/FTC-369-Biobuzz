package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    public DcMotorEx fly1;
    public DcMotorEx fly2;
    public DcMotorEx turret;

    // Assumes PIDF_Controller is a valid user-defined class in your package
    PIDF_Controller pid;

    private double m_robotRelativeAngle;
    private double m_fieldRelativeAngle;
    private double flyPower;

    // Physical offset of the turret pivot point relative to the robot center (inches)
    private final double offsetX = 0.0;
    private final double offsetY = 0.0;

    // Physical Gearbox Math
    private final double totalTurnTicks = 853.0;
    private final double totalDegrees = 320.0;
    private final double ticksPerDegree = totalTurnTicks / totalDegrees;

    boolean useRegression = false;

    public Turret(HardwareMap hardwareMap) {
        // 1. Initialize Flywheels
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        flyPower = 0;
        fly2.setDirection(DcMotorSimple.Direction.REVERSE);

        // 2. Initialize Turret Pivot Motor with Safe Mode Sequence
        turret = hardwareMap.get(DcMotorEx.class, "turret");


        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        // Clean mode switching sequence to prevent instant lockup / jumps
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.0); // Keep power at zero until a tracking target is commanded

        // NOTE: Make sure to tune these constants inside your PIDF_Controller class
        pid = new PIDF_Controller(1.0, 0.0, 0.0, 0.0);
    }

    public void setShooterMode() {
        useRegression = !useRegression;
    }

    public void setTurretPower(double distance) {
        if (useRegression) {
            regression(distance);
        } else {
            fly1.setVelocity(flyPower);
            fly2.setVelocity(flyPower);
        }
    }

    public void setTurretPower(double distance, double increment) {
        if (useRegression) {
            regression(distance);
        } else {
            flyPower += increment;
            fly1.setVelocity(flyPower);
            fly2.setVelocity(flyPower);
        }
    }

    /**
     * Tracks a field-centric position using Road Runner geometry.
     * Assumes target and robotPose share the exact same global field coordinate frame.
     */
    public void turretTrackPosition(Pose2d target, Pose2d robotPose) {
        // Convert offset into a vector
        Vector2d turretOffsetVector = new Vector2d(offsetX, offsetY);

        // Rotate the turret's physical offset by the robot's heading and add it to its position
        Vector2d turretFieldPosition = robotPose.position.plus(robotPose.heading.times(turretOffsetVector));

        // Get delta coordinates from turret to target
        double dX = target.position.x - turretFieldPosition.x;
        double dY = target.position.y - turretFieldPosition.y;

        // Calculate absolute field angle to target (Radians)
        double fieldAngleRadians = Math.atan2(dY, dX);

        // Extract robot heading correctly from Road Runner v1.0 Rotation2d
        double robotHeadingRadians = robotPose.heading.toDouble();

        // Target angle relative to the front of the robot chassis
        double robotRelativeAngleRadians = fieldAngleRadians - robotHeadingRadians;

        // Normalize angle to the range [-PI, PI] to prevent infinite 360-degree windup wraps
        while (robotRelativeAngleRadians > Math.PI) robotRelativeAngleRadians -= 2 * Math.PI;
        while (robotRelativeAngleRadians < -Math.PI) robotRelativeAngleRadians += 2 * Math.PI;

        m_fieldRelativeAngle = Math.toDegrees(fieldAngleRadians);
        m_robotRelativeAngle = Math.toDegrees(robotRelativeAngleRadians);

        // Command the motor to target degree angle
        this.setPivotPosition(m_robotRelativeAngle);
    }

    private void setPivotPosition(double degrees) {
        // Physical hard-stop safeguards for your mechanical limits
        double clampedDegrees = Math.max(-150.0, Math.min(150.0, degrees));
        int targetTicks = (int) (clampedDegrees * ticksPerDegree);

        // Set the target first, then enforce the mode, then supply the power
        turret.setTargetPosition(targetTicks);
        if (turret.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        turret.setPower(0.6); // Lowered slightly from 0.8 for testing safety
    }

    public void regression(double distanceToHive) {
        double targetVelocity = 10.31616 * distanceToHive + 945.07026;
        pid.setTarget(targetVelocity);

        fly1.setVelocity(pid.calculate(fly1.getVelocity()));
        fly2.setVelocity(pid.calculate(fly2.getVelocity()));
    }

    public double getFlyPower() {
        return flyPower;
    }

    public String getShooterMode() {
        return useRegression ? "Auto" : "Manual";
    }
}
