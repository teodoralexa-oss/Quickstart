package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.geometry.Pose;

public class TurretSubsystem {
    public DcMotorEx turret;
    double goalX = 144;
    double goalY = 144;
    double ticksPerRev = 537.7;
    double gearRatio = 5.0;
    double ticksPerRad = (ticksPerRev * gearRatio) / (2 * Math.PI);
    double maxAngle = Math.toRadians(150);
    double minAngle = Math.toRadians(-150);

    public TurretSubsystem(HardwareMap hw) {
        turret = hw.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPower(0);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setBlueGoal() {
        goalX = 144;
        goalY = 144;
    }

    public void setRedGoal() {
        goalX = 0;
        goalY = 0;
    }

    double normAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public void update(Pose pose) {
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading();
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double targetAngle = Math.atan2(dy, dx);
        double turretAngle = normAngle(targetAngle - robotHeading);
        turretAngle = applyWrapLogic(turretAngle);
        int targetTicks = (int) (turretAngle * ticksPerRad);
        turret.setTargetPosition(targetTicks);
        turret.setPower(0.7);
    }

    double applyWrapLogic(double angle) {
        if (angle > maxAngle) angle -= 2 * Math.PI;
        if (angle < minAngle) angle += 2 * Math.PI;
        return angle;
    }
}