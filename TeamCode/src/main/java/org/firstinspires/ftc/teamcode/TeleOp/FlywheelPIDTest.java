package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@TeleOp(name="Flywheel PIDF Test")
public class FlywheelPIDTest extends LinearOpMode {

    DcMotorEx flywheel;

    double targetRPM = 3000;

    double P = 15;
    double I = 0;
    double D = 0;
    double F = 11.5;

    @Override
    public void runOpMode() {

        flywheel = hardwareMap.get(DcMotorEx.class, "turret");
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheel.setVelocityPIDFCoefficients(P, I, D, F);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) targetRPM += 50;
            if (gamepad1.dpad_down) targetRPM -= 50;

            if (gamepad1.a) P += 0.5;
            if (gamepad1.b) P -= 0.5;

            if (gamepad1.x) F += 0.1;
            if (gamepad1.y) F -= 0.1;

            flywheel.setVelocity(rpmToTicks(targetRPM));

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual RPM", ticksToRPM(flywheel.getVelocity()));
            telemetry.addData("P", P);
            telemetry.addData("F", F);
            telemetry.update();

            flywheel.setVelocityPIDFCoefficients(P, I, D, F);
        }
    }

    double rpmToTicks(double rpm) {
        return rpm * 28 / 60.0;
    }

    double ticksToRPM(double ticks) {
        return ticks * 60.0 / 28.0;
    }
}