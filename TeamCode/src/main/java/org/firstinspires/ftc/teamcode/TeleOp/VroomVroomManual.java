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
    public DcMotorEx motion2;

    private double currentAngle = 115.0;
    private static final double ANGLE_STEP = 5.0;
    private static final double MIN_ANGLE = 90.0;
    private static final double MAX_ANGLE = 140.0;
    private static final double PRESET_LOW = 90.0;
    private static final double PRESET_HIGH = 140.0;

    private boolean yPressedLast = false;
    private boolean aPressedLast = false;
    private boolean xPressedLast = false;
    private boolean bPressedLast = false;

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
        motion2 = hardwareMap.get(DcMotorEx.class, "motion2");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            moveDriveTrain();
            updateRampServos();

            if (gamepad1.dpad_down) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }

            if (gamepad1.dpad_up) {
                outtake.setPower(1.0);
            } else {
                outtake.setPower(0.0);
            }

            if (gamepad1.dpad_right) {
                motion.setPower(1.0);
                motion2.setPower(1.0);
            } else {
                motion.setPower(0.0);
                motion2.setPower(0.0);
            }

            sleep(20);
        }
    }

    private void updateRampServos() {
        if (gamepad1.y) {
            currentAngle = PRESET_HIGH;
            //yPressedLast = true;
        }

        if (gamepad1.a) {
            currentAngle = PRESET_LOW;
            //aPressedLast = true;
        }
        if (gamepad1.x) {
            currentAngle -= ANGLE_STEP;
            if (currentAngle < MIN_ANGLE) currentAngle = MIN_ANGLE;
            //xPressedLast = true;
        }

        if (gamepad1.b) {
            currentAngle += ANGLE_STEP;
            if (currentAngle > MAX_ANGLE) currentAngle = MAX_ANGLE;
            //bPressedLast = true;
        }

        double servoPos = currentAngle / 180.0;
        if (servoPos < 0.0) servoPos = 0.0;
        if (servoPos > 1.0) servoPos = 1.0;
        servo_ramp_left.setPosition(servoPos);
        servo_ramp_right.setPosition(servoPos);
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