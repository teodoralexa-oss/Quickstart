package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.TeleOp.FlywheelPIDTest;

@TeleOp(name = "ShooterIntake")
public class Core extends LinearOpMode {

    DcMotorEx intake1, intake2;
    Servo barrier2, slider;
    FlywheelPIDTest flywheel = new FlywheelPIDTest();

    double targetRPMfar = 2350;
    double targetRPMclose = 2100;

    @Override
    public void runOpMode() {

        intake1 = hardwareMap.get(DcMotorEx.class, "int1");
        intake2 = hardwareMap.get(DcMotorEx.class, "int2");
        barrier2 = hardwareMap.get(Servo.class, "b2");
        slider = hardwareMap.get(Servo.class, "sld");
        intake1.setDirection(DcMotorEx.Direction.FORWARD);
        intake2.setDirection(DcMotorEx.Direction.REVERSE);

        intake1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        flywheel.init(hardwareMap, telemetry);

        telemetry.addLine("READY - ShooterIntake");
        telemetry.update();
        waitForStart();
        barrier2.setPosition(0.5);  // inchis la start


        while (opModeIsActive()) {

            if (gamepad1.dpad_right) {
                flywheel.targetRPM = targetRPMfar;
            } else if (gamepad1.dpad_left) {
                flywheel.targetRPM = targetRPMclose;
            }
            if (gamepad1.square){
                slider.setPosition (1);

            }
            else if (gamepad1.triangle){
            slider.setPosition(0);
            }



            flywheel.loop(gamepad1.left_bumper);

            if (gamepad1.right_bumper) {
                intake1.setPower(1);
                intake2.setPower(1);
            } else if (gamepad1.cross) {
                intake2.setPower(-1);
            } else {
                intake1.setPower(0);
                intake2.setPower(0);
            }

            if(gamepad1.dpad_down){
                barrier2.setPosition(0.5);
            }else if(gamepad1.right_trigger>0.2){
                barrier2.setPosition(0.80);
            }

            telemetry.addData("Target RPM", flywheel.targetRPM);
            telemetry.addData("Barrier2", barrier2.getPosition());
            telemetry.addData("Intake1 Power", intake1.getPower());
            telemetry.addData("Intake2 Power", intake2.getPower());
            telemetry.update();
        }
    }
}