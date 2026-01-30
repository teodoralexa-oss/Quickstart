package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TeleOp.AprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Shooter {

    private DcMotorEx turret;
    private AprilTag aprilTag;

    private final double kP = 0.015;
    private final double maxPower = 0.5;
    private final double deadZone = 0.8;

    private final double searchPower = 0.2;
    private int searchDirection = 1;

    public void init(HardwareMap hwMap, AprilTag aprilTag) {
        this.aprilTag = aprilTag;
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void update(int targetId) {
        AprilTagDetection tag = aprilTag.getTagBySpecificid(targetId);

        if (tag == null) {
            turret.setPower(searchPower * searchDirection);
            return;
        }

        double error = tag.ftcPose.bearing;

        if (Math.abs(error) < deadZone) {
            turret.setPower(0);
            return;
        }

        double power = error * kP;

        if (power > maxPower) power = maxPower;
        if (power < -maxPower) power = -maxPower;

        turret.setPower(power);
    }

    public void reverseSearchDirection() {
        searchDirection *= -1;
    }

    public void stop() {
        turret.setPower(0);
    }
}
