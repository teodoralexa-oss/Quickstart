package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class VroomVroomManual extends LinearOpMode {
    //---MOTORS---\\
    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;
    public DcMotorEx outtake1;
    public DcMotorEx outtake2;
    //---SERVO---\\
    public CRServo intake1left;
    public CRServo intake1right;
    public CRServo intake2left;
    public CRServo intake2right;
    public CRServo intake3left;
    public CRServo intake3right;
    //---OTHER---\\
    double ticks = 2786.2;
    double newTarget;
    @Override
    public void runOpMode() throws InterruptedException {
        //---MOTORS---\\
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "stangaFataMotor");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "dreaptaFataMotor");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "stangaSpateMotor");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "dreaptaSpateMotor");
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        //---SERVO---\\
        intake1left = hardwareMap.get(CRServo.class, "servo1");
        intake1right = hardwareMap.get(CRServo.class, "servo2");
        intake2left = hardwareMap.get(CRServo.class, "servo3");
        intake2right = hardwareMap.get(CRServo.class, "servo4");
        intake3left = hardwareMap.get(CRServo.class, "servo5");
        intake3right = hardwareMap.get(CRServo.class, "servo6");
        //---LOGIC---\\
        outtake1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            moveDriveTrain();

            if(gamepad1.a){
                intake1left.setPower(-1);
                intake1right.setPower(1);
                intake2left.setPower(-1);
                intake2right.setPower(1);
                intake3left.setPower(1);
                intake3right.setPower(-1);
            }
            if(gamepad1.circle){
                intake1left.setPower(0);
                intake1right.setPower(0);
                intake2left.setPower(0);
                intake2right.setPower(0);
                intake3left.setPower(0);
                intake3right.setPower(0);
            }

            if (gamepad1.right_bumper) {
                Encoder(2);
            }

            //sleep(20);
        }
    }
    public void Encoder(int turnage){
        outtake1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        newTarget=turnage/ticks;
        outtake1.setTargetPosition((int)newTarget);
        outtake1.setPower(0.6);
        outtake1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void moveDriveTrain() {
        double vertical = -gamepad1.left_stick_y;
        double horizontal = gamepad1.right_stick_x;
        double turn = gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(turn), 1);

        leftFrontMotor.setPower((vertical + horizontal + turn) / denominator);
        leftBackMotor.setPower((vertical - horizontal + turn) / denominator);
        rightFrontMotor.setPower((vertical - horizontal - turn) / denominator);
        rightBackMotor.setPower((vertical + horizontal - turn) / denominator);
    }
}