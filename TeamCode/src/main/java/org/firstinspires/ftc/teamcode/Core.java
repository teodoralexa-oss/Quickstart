package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOp.AprilTag;
import org.firstinspires.ftc.teamcode.TeleOp.FlywheelPIDTest;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@TeleOp
public class Core extends LinearOpMode {

    DcMotorEx leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    DcMotorEx intake1, intake2;
    Servo barrier1;
    Servo barrier2;
    Servo rampServo;

    AprilTag aprilTag = new AprilTag();
    FlywheelPIDTest flywheel = new FlywheelPIDTest();
    GoBildaPinpointDriver pinpoint;

    double kp = 0.011;
    double kd = 0.0006;
    double goalX = 0;
    double angleTolerance = 0.5;

    double error = 0;
    double lastError = 0;
    double lastTime = 0;

    Integer targetTagId = null;
    AprilTagDetection currentTag = null;

    static final double MM_TO_INCHES = 1.0 / 25.4;

    double blueGoalX = 0.0;
    double blueGoalY = 0.0;
    double redGoalX = 144.0;
    double redGoalY = 144.0;

    double currentGoalX = 0.0;
    double currentGoalY = 0.0;

    double targetRPMfar = 2120;
    double targetRPMclose = 1850;

    boolean driveReversed = false;

    @Override
    public void runOpMode() {

        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "stangaFataMotor");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "dreaptaFataMotor");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "stangaSpateMotor");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "dreaptaSpateMotor");
        barrier1 = hardwareMap.get(Servo.class, "b1");
        barrier2 = hardwareMap.get(Servo.class, "b2");
        rampServo = hardwareMap.get(Servo.class, "rampServo");

        intake1 = hardwareMap.get(DcMotorEx.class, "int1");
        intake2 = hardwareMap.get(DcMotorEx.class, "int2");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0.0, 2.0, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intake1.setDirection(DcMotorEx.Direction.FORWARD);
        intake2.setDirection(DcMotorEx.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        aprilTag.init(hardwareMap, telemetry);
        flywheel.init(hardwareMap, telemetry);

        telemetry.addLine("READY");
        telemetry.update();
        waitForStart();
        lastTime = getRuntime();
        barrier2.setPosition(0.5);

        try {
            while (opModeIsActive()) {

                pinpoint.update();

                if (gamepad1.triangleWasPressed()) {
                    driveReversed = !driveReversed;
                }

                if (gamepad1.squareWasPressed()) {
                    targetTagId = 20;
                    currentGoalX = blueGoalX;
                    currentGoalY = blueGoalY;
                    lastError = 0;
                    telemetry.addLine("Target TAG: 20 (BLUE)");
                }

                if (gamepad1.circleWasPressed()) {
                    targetTagId = 24;
                    currentGoalX = redGoalX;
                    currentGoalY = redGoalY;
                    lastError = 0;
                    telemetry.addLine("Target TAG: 24 (RED)");
                }

                aprilTag.update();

                if (targetTagId != null) {
                    currentTag = aprilTag.getTagBySpecificid(targetTagId);
                } else {
                    currentTag = null;
                }

                double robotX = pinpoint.getPosX(DistanceUnit.MM) * MM_TO_INCHES;
                double robotY = pinpoint.getPosY(DistanceUnit.MM) * MM_TO_INCHES;

                double distance = getDistanceToGoal(robotX, robotY);

                if (gamepad1.dpad_right) {
                    flywheel.targetRPM = targetRPMfar;
                } else if (gamepad1.dpad_left) {
                    flywheel.targetRPM = targetRPMclose;
                }

                rampServo.setPosition(0);

                double vertical = -gamepad1.left_stick_y;
                double horizontal = gamepad1.left_stick_x;
                double manualTurn = gamepad1.right_stick_x;

                if (driveReversed) {
                    vertical = -vertical;
                    horizontal = -horizontal;
                }

                double autoTurn = 0;

                if (gamepad1.left_trigger > 0.3 && currentTag != null) {
                    error = goalX - currentTag.ftcPose.bearing;

                    if (Math.abs(error) > angleTolerance) {
                        double now = getRuntime();
                        double dt = now - lastTime;

                        if (dt > 0) {
                            autoTurn = Range.clip(
                                    kp * error + kd * (error - lastError) / dt,
                                    -0.4, 0.4
                            );
                        }

                        lastError = error;
                        lastTime = now;
                    }
                } else {
                    lastError = 0;
                    lastTime = getRuntime();
                }

                double turn = manualTurn + autoTurn;

                double denom = Math.max(
                        Math.abs(vertical) + Math.abs(horizontal) + Math.abs(turn),
                        1
                );

                leftFrontMotor.setPower((vertical + horizontal + turn) / denom);
                leftBackMotor.setPower((vertical - horizontal + turn) / denom);
                rightFrontMotor.setPower((vertical - horizontal - turn) / denom);
                rightBackMotor.setPower((vertical + horizontal - turn) / denom);

                flywheel.loop(gamepad1.left_bumper);

                if(gamepad1.dpad_down){
                    barrier2.setPosition(0.5);
                }else if(gamepad1.right_trigger>0.2){
                    barrier2.setPosition(0.73);
                }

                if (gamepad1.right_bumper) {
                    intake1.setPower(1);
                    intake2.setPower(1);
                } else if (gamepad1.cross) {
                    //intake1.setPower(-1);
                    intake2.setPower(-1);
                } else {
                    intake1.setPower(0);
                    intake2.setPower(0);
                }

                telemetry.addData("Drive Mode", driveReversed ? "REVERSED (Intake Back)" : "NORMAL");
                telemetry.addData("Aimbot",
                        gamepad1.left_trigger > 0.3 && currentTag != null ? "ON" : "OFF");
                telemetry.addData("Target Tag", targetTagId);
                telemetry.addData("Tag Visible", currentTag != null);
                telemetry.addData("Bearing",
                        currentTag != null ? currentTag.ftcPose.bearing : 0);
                telemetry.addData("Distance", distance);
                telemetry.addData("Target RPM", flywheel.targetRPM);
                telemetry.addData("Robot X (in)", robotX);
                telemetry.addData("Robot Y (in)", robotY);
                telemetry.update();
            }

        } finally {
            aprilTag.stop();
        }
    }

    double getDistanceToGoal(double robotX, double robotY) {
        if (targetTagId == null) return 0;

        double dx = robotX - currentGoalX;
        double dy = robotY - currentGoalY;

        return Math.sqrt(dx * dx + dy * dy);
    }

    double map(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
        return toLow + (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow);
    }
}