package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@Autonomous
public class MecanumAutoo extends LinearOpMode {
    //---MOTORS---\\
    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;
    public DcMotorEx outtake1;
    public DcMotorEx outtake2;
    boolean go=true;

    @Override
    public void runOpMode() throws InterruptedException {
        //---MOTORS---\\
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "stangaFataMotor");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "dreaptaFataMotor");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "stangaSpateMotor");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "dreaptaSpateMotor");
        outtake1 = hardwareMap.get(DcMotorEx.class, "l1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "l2");
        //outtake1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake1.setDirection(DcMotorEx.Direction.REVERSE);
        MotorConfigurationType motor = outtake1.getMotorType();
        motor.setAchieveableMaxRPMFraction(1.0);
        outtake1.setMotorType(motor);
        MotorConfigurationType motor2 = outtake2.getMotorType();
        motor.setAchieveableMaxRPMFraction(1.0);
        outtake2.setMotorType(motor2);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            if(go){
                sleep(25000);
                moove();
                sleep(1000);
                end();
                go=false;
            }
        }
    }
    public void shoot(){
        double speed=0.0;
        while(speed<=1.0){
            speed+=0.25;
            outtake1.setPower(speed);
            outtake2.setPower(speed);
        }
        sleep(1000);
        outtake1.setPower(-0.7);
        outtake2.setPower(-0.7);
        sleep(500);
    }
    public void strafeRight(){
        leftFrontMotor.setPower(0.5);
        rightFrontMotor.setPower(-0.5);
        leftBackMotor.setPower(-0.5);
        rightBackMotor.setPower(0.5);
    }
    public void moove(){
        leftFrontMotor.setPower(0.6);
        rightFrontMotor.setPower(0.6);
    }
    public void end(){
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
}