package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
    public DcMotorEx intake1;
    //---SERVO---\\
    public CRServo intake1left;
    public CRServo intake1right;
    public CRServo intake2left;
    public CRServo intake2right;
    public CRServo intake3left;
    public CRServo intake3right;

    @Override
    public void runOpMode() throws InterruptedException {
        //---MOTORS---\\
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "stangaFataMotor");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "dreaptaFataMotor");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "stangaSpateMotor");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "dreaptaSpateMotor");
        intake1 = hardwareMap.get(DcMotorEx.class, "motion");
        //---SERVO---\\
        intake1left = hardwareMap.get(CRServo.class, "servo1");
        intake1right = hardwareMap.get(CRServo.class, "servo2");
        intake2left = hardwareMap.get(CRServo.class, "servo3");
        intake2right = hardwareMap.get(CRServo.class, "servo4");
        intake3left = hardwareMap.get(CRServo.class, "servo5");
        intake3right = hardwareMap.get(CRServo.class, "servo6");
        //---LOGIC---\\
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            moveDriveTrain();

            if(gamepad1.cross){
                intake1left.setPower(4);
                intake1right.setPower(-4);
                intake2left.setPower(4);
                intake2right.setPower(-4);
                intake3left.setPower(4);
                intake3right.setPower(-4);
            }
            if(gamepad1.circle){
                intake1left.setPower(0);
                intake1right.setPower(0);
                intake2left.setPower(0);
                intake2right.setPower(0);
                intake3left.setPower(1);
                intake3right.setPower(-1);
            }

            if (gamepad1.right_bumper) {
                intake1.setPower(1.0);

            } else {
                intake1.setPower(0.0);
            }

            //sleep(20);
        }
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