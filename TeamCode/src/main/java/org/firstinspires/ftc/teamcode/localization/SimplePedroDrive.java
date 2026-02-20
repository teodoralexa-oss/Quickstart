package org.firstinspires.ftc.teamcode.localization;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SimplePedroDrive {

    BNO055IMU imu;

    double x = 72;
    double y = 72;
    double heading = 0;

    public SimplePedroDrive(HardwareMap hardwareMap) {

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(params);
    }

    public void update() {
        heading = imu.getAngularOrientation().firstAngle;
    }

    public Pose getPose() {
        return new Pose(x, y, heading);
    }

    public void setPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}