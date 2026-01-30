package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Flywheel PIDF Test")
public class FlywheelPIDTest extends LinearOpMode {

    DcMotorEx fly;

    double targetRPM = 6000;

    double kP = 0.00035;
    double kI = 0.000001;
    double kD = 0.00001;
    double kF = 0.00017;

    double lastError = 0;
    double integralSum = 0;

    @Override
    public void runOpMode() {

        fly = hardwareMap.get(DcMotorEx.class, "l1");
        fly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) targetRPM = 6000;
            if (gamepad1.b) targetRPM = 0;

            double velocity = fly.getVelocity();
            double currentRPM = velocity * 60 / 28;

            double error = targetRPM - currentRPM;
            integralSum += error;
            double derivative = error - lastError;

            double power =
                    kP * error +
                            kI * integralSum +
                            kD * derivative +
                            kF * targetRPM;

            power = Math.max(0, Math.min(1, power));

            fly.setPower(power);

            lastError = error;

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", currentRPM);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}
