package org.firstinspires.ftc.teamcode.EncAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class BlueFarEAuto extends LinearOpMode {
    //---MOTORS---\\
    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;
    boolean go=true;
    double ticks = 384.5;
    double newTarget;

    @Override
    public void runOpMode() throws InterruptedException {
        //---MOTORS---\\
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "stangaFataMotor");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "dreaptaFataMotor");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "stangaSpateMotor");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "dreaptaSpateMotor");
        //---LOGIC---\\
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            if(go){

            }
        }
    }
    public void Forward(int turnage){
        newTarget=ticks/turnage;
        leftFrontMotor.setTargetPosition((int)newTarget);
        rightFrontMotor.setTargetPosition((int)newTarget);
        leftBackMotor.setTargetPosition((int)newTarget);
        rightBackMotor.setTargetPosition((int)newTarget);

        leftFrontMotor.setPower(0.5);
        rightFrontMotor.setPower(0.5);
        rightBackMotor.setPower(0.5);
        leftBackMotor.setPower(0.5);

        leftFrontMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public void Back(int turnage){
        newTarget=ticks/turnage;
        leftFrontMotor.setTargetPosition((int)newTarget);
        rightFrontMotor.setTargetPosition((int)newTarget);
        leftBackMotor.setTargetPosition((int)newTarget);
        rightBackMotor.setTargetPosition((int)newTarget);
        leftFrontMotor.setPower(-0.5);
        rightFrontMotor.setPower(-0.5);
        rightBackMotor.setPower(-0.5);
        leftBackMotor.setPower(-0.5);

        leftFrontMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

}