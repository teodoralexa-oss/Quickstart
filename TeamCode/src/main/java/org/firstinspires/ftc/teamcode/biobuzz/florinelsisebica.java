package org.firstinspires.ftc.teamcode.biobuzz;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;0

@TeleOp
public class florinelsisebica extends LinearOpMode {
    DcMotorEx leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Sebica si Florinel ready for action");
        telemetry.update();
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "stangaFataMotor");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "dreaptaFataMotor");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "stangaSpateMotor");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "dreaptaSpateMotor");

        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftFrontMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            double vertical = -gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double manualTurn = gamepad1.right_stick_x;

            double turn = manualTurn;

            double denom = Math.max(
                    Math.abs(vertical) + Math.abs(horizontal) + Math.abs(turn),
                    1
            );

            leftFrontMotor.setPower((vertical + horizontal + turn) / denom);
            leftBackMotor.setPower((vertical - horizontal + turn) / denom);
            rightFrontMotor.setPower((vertical - horizontal - turn) / denom);
            rightBackMotor.setPower((vertical + horizontal - turn) / denom);
        }
    }
}