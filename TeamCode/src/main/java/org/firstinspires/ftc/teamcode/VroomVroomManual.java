package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Auto.Sorter;
import org.firstinspires.ftc.teamcode.TeleOp.AprilTag;

@TeleOp
public class VroomVroomManual extends LinearOpMode {

    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;
    public DcMotorEx outtake1;

    AprilTag aprilTag = new AprilTag();
    Sorter sorter = new Sorter();

    @Override
    public void runOpMode() {

        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "stangaFataMotor");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "dreaptaFataMotor");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "stangaSpateMotor");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "dreaptaSpateMotor");
        outtake1 = hardwareMap.get(DcMotorEx.class, "l1");

        outtake1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        outtake1.setDirection(DcMotorEx.Direction.REVERSE);

        MotorConfigurationType motor = outtake1.getMotorType();
        motor.setAchieveableMaxRPMFraction(1.0);
        outtake1.setMotorType(motor);

        aprilTag.init(hardwareMap, telemetry);
        sorter.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            aprilTag.update();

            sorter.update(gamepad2.dpad_left, gamepad2.dpad_right);

            moveDriveTrain();

            double launchStick = gamepad2.left_stick_y;
            outtake1.setPower(-launchStick);

            telemetry.update();
        }
    }

    public void moveDriveTrain() {
        double vertical = -gamepad1.left_stick_y;
        double horizontal = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(turn), 1);

        leftFrontMotor.setPower((vertical + horizontal + turn) / denominator);
        leftBackMotor.setPower((vertical - horizontal + turn) / denominator);
        rightFrontMotor.setPower((vertical - horizontal - turn) / denominator);
        rightBackMotor.setPower((vertical + horizontal - turn) / denominator);
    }
}
