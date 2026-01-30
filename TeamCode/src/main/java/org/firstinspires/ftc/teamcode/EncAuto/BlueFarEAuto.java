package org.firstinspires.ftc.teamcode.EncAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class BlueFarEAuto extends LinearOpMode {

    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;
    public DcMotorEx outtake1;
    public DcMotorEx outtake2;

    boolean go = true;

    double ticks = 384.5;
    double newTarget;
    double strafeFactor = 1.1;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "stangaFataMotor");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "dreaptaFataMotor");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "stangaSpateMotor");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "dreaptaSpateMotor");
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtake2.setDirection(DcMotorEx.Direction.REVERSE);
        outtake1.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (go) {
                path();
                go = false;
            }
        }
    }

    public void path() {
        Forward(1);
    }
    public void Shoot(){
        outtake1.setPower(1.0);
        outtake2.setPower(-1.0);
        sleep(500);
        outtake1.setPower(0);
        outtake2.setPower(0);
    }
    public void TurnLeft(double turnage){
        ResetEncoders();
        newTarget = ticks * turnage;

        leftFrontMotor.setTargetPosition((int) newTarget);
        rightFrontMotor.setTargetPosition((int) -newTarget);
        leftBackMotor.setTargetPosition((int) -newTarget);
        rightBackMotor.setTargetPosition((int) newTarget);

        leftFrontMotor.setPower(0.5);
        rightFrontMotor.setPower(0.5);
        leftBackMotor.setPower(0.5);
        rightBackMotor.setPower(0.5);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() &&
                leftFrontMotor.isBusy() &&
                rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() &&
                rightBackMotor.isBusy()) {}
    }
    public void Forward(double turnage) {
        ResetEncoders();
        newTarget = ticks * turnage;

        leftFrontMotor.setTargetPosition((int) newTarget);
        rightFrontMotor.setTargetPosition((int) newTarget);
        leftBackMotor.setTargetPosition((int) newTarget);
        rightBackMotor.setTargetPosition((int) newTarget);

        leftFrontMotor.setPower(0.5);
        rightFrontMotor.setPower(0.5);
        leftBackMotor.setPower(0.5);
        rightBackMotor.setPower(0.5);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() &&
                leftFrontMotor.isBusy() &&
                rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() &&
                rightBackMotor.isBusy()) {}
    }

    public void Back(double turnage) {
        ResetEncoders();
        newTarget = ticks * turnage;

        leftFrontMotor.setTargetPosition((int) -newTarget);
        rightFrontMotor.setTargetPosition((int) -newTarget);
        leftBackMotor.setTargetPosition((int) -newTarget);
        rightBackMotor.setTargetPosition((int) -newTarget);

        leftFrontMotor.setPower(0.5);
        rightFrontMotor.setPower(0.2);
        leftBackMotor.setPower(0.2);
        rightBackMotor.setPower(0.5);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() &&
                leftFrontMotor.isBusy() &&
                rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() &&
                rightBackMotor.isBusy()) {}
    }

    public void StrafeRight(double turnage) {
        ResetEncoders();
        newTarget = ticks * turnage * strafeFactor;

        leftFrontMotor.setTargetPosition((int) newTarget);
        rightFrontMotor.setTargetPosition((int) -newTarget);
        leftBackMotor.setTargetPosition((int) -newTarget);
        rightBackMotor.setTargetPosition((int) newTarget);

        leftFrontMotor.setPower(0.5);
        rightFrontMotor.setPower(0.5);
        leftBackMotor.setPower(0.5);
        rightBackMotor.setPower(0.5);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() &&
                leftFrontMotor.isBusy() &&
                rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() &&
                rightBackMotor.isBusy()) {}
    }

    public void StrafeLeft(double turnage) {
        ResetEncoders();
        newTarget = ticks * turnage * strafeFactor;

        leftFrontMotor.setTargetPosition((int) -newTarget);
        rightFrontMotor.setTargetPosition((int) newTarget);
        leftBackMotor.setTargetPosition((int) newTarget);
        rightBackMotor.setTargetPosition((int) -newTarget);

        leftFrontMotor.setPower(0.5);
        rightFrontMotor.setPower(0.5);
        leftBackMotor.setPower(0.5);
        rightBackMotor.setPower(0.5);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() &&
                leftFrontMotor.isBusy() &&
                rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() &&
                rightBackMotor.isBusy()) {}
    }

    public void ResetEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
