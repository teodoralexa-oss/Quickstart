package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class MecanumAuto extends LinearOpMode {
    //---MOTORS---\\
    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;
    boolean go=true;

    @Override
    public void runOpMode() throws InterruptedException {
        //---MOTORS---\\
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "stangaFataMotor");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "dreaptaFataMotor");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "stangaSpateMotor");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "dreaptaSpateMotor");

        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            if(go){
                moove();
                sleep(1000);
                end();
                go=false;
            }
        }
    }
    public void moove() {
        leftFrontMotor.setPower(0.4);
        rightFrontMotor.setPower(0.4);
    }
    public void end(){
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }
}