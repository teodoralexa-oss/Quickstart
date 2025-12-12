package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="RobotVroomVroom")
public class VroomVroomManual extends LinearOpMode {
    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;
    public Servo servo_ramp_right;
    public Servo servo_ramp_left;
    public DcMotorEx intake;
    public DcMotorEx outtake;
    public DcMotorEx motion;
    //0 intake, 1 outtake, 2 motion
    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "stangaFataMotor");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "dreaptaFataMotor");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "stangaSpateMotor");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "dreaptaSpateMotor");
        servo_ramp_right = hardwareMap.get(Servo.class, "servo1");
        servo_ramp_left = hardwareMap.get(Servo.class, "servo2");
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        motion = hardwareMap.get(DcMotorEx.class, "motion");


        waitForStart();
        servo_ramp_right.setPosition(0);
        servo_ramp_left.setPosition(0);
        while (opModeIsActive() && !isStopRequested()) {
            moveDriveTrain();

            if(gamepad1.y){
                servo_ramp_right.setPosition(-0.1);
                servo_ramp_left.setPosition(0.1);
            }
            if(gamepad1.a){
                servo_ramp_right.setPosition(0.1);
                servo_ramp_left.setPosition(-0.1);
            }
            if(gamepad1.dpad_down){
                intake.setPower(1);
            }
            intake.setPower(0);
            if(gamepad1.dpad_up){
                outtake.setPower(1);
            }
            outtake.setPower(0);
            if(gamepad1.dpad_right){
                motion.setPower(1);
            }
            motion.setPower(0);

        }
    }
    public void moveDriveTrain(){
        double vertical = 0;
        double horizontal = 0;
        double turn = 0;

        vertical = -gamepad1.left_stick_y;
        horizontal = gamepad1.right_stick_x;
        turn = gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(turn), 1);

        leftFrontMotor.setPower((vertical + horizontal + turn) / denominator);
        leftBackMotor.setPower((vertical - horizontal + turn) / denominator);
        rightFrontMotor.setPower((vertical - horizontal - turn) / denominator);
        rightBackMotor.setPower((vertical + horizontal - turn) / denominator);

    }
}