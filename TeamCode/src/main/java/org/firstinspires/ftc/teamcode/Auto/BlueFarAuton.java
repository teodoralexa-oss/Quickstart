package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TeleOp.FlywheelPIDTest;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class BlueFarAuton extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, shootTimer, waitTimer;
    private int pathState;

    private Pose startPose = new Pose(58, 13, Math.toRadians(25));
    private Pose shootPose = new Pose(53, 30, Math.toRadians(36));
    private Pose basePose = new Pose(18, 44, Math.toRadians(180));
    private Pose donePose = new Pose(69, 42, Math.toRadians(180));

    private DcMotorEx intake1;
    private DcMotorEx intake2;
    private DcMotorEx outtake1;
    private DcMotorEx outtake2;
    private Servo rampServo;
    private Servo barrier1;
    private Servo barrier2;
    private FlywheelPIDTest flywheel;

    private Path start_shoot, shoot_base, base_shoot, shoot_done;

    public void buildPaths() {
        start_shoot = new Path(new BezierLine(startPose, shootPose));
        start_shoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        shoot_base = new Path(new BezierLine(shootPose, basePose));
        shoot_base.setLinearHeadingInterpolation(startPose.getHeading(),basePose.getHeading());

        base_shoot = new Path(new BezierLine(basePose, shootPose));
        base_shoot.setLinearHeadingInterpolation(basePose.getHeading(), shootPose.getHeading());

        shoot_done = new Path(new BezierLine(shootPose, donePose));
        shoot_done.setLinearHeadingInterpolation(shootPose.getHeading(), donePose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(start_shoot, true);
                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    flywheel.targetRPM = 2100;
                    flywheel.loop(true);
                    waitTimer.resetTimer();
                    setPathState(2);
                }
                break;

            case 2:
                flywheel.loop(true);
                if (waitTimer.getElapsedTimeSeconds() > 1.7 && flywheel.isAtSpeed(100)) {
                    startIntake();
                    shootTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                flywheel.loop(true);
                if (shootTimer.getElapsedTimeSeconds() > 3.5) {
                    stopIntake();
                    stopFlywheelAndRelease();
                    follower.followPath(shoot_base, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    startRowIntake();
                    follower.setMaxPower(0.5);
                    waitTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5:
                if (waitTimer.getElapsedTimeSeconds() > 2.0) {
                    stopRowIntake();
                    follower.setMaxPower(1.0);
                    follower.followPath(base_shoot, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    flywheel.targetRPM = 2100;
                    flywheel.loop(true);
                    waitTimer.resetTimer();
                    setPathState(7);
                }
                break;

            case 7:
                flywheel.loop(true);
                if (waitTimer.getElapsedTimeSeconds() > 1.0 && flywheel.isAtSpeed(100)) {
                    startIntake();
                    shootTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8:
                flywheel.loop(true);
                if (shootTimer.getElapsedTimeSeconds() > 3.0) {
                    stopIntake();
                    stopFlywheelAndRelease();
                    follower.followPath(shoot_base, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    startRowIntake();
                    follower.setMaxPower(0.5);
                    waitTimer.resetTimer();
                    setPathState(10);
                }
                break;

            case 10:
                if (waitTimer.getElapsedTimeSeconds() > 2.0) {
                    stopRowIntake();
                    follower.setMaxPower(1.0);
                    follower.followPath(base_shoot, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    flywheel.targetRPM = 1700;
                    flywheel.loop(true);
                    waitTimer.resetTimer();
                    setPathState(12);
                }
                break;

            case 12:
                flywheel.loop(true);
                if (waitTimer.getElapsedTimeSeconds() > 1.0 && flywheel.isAtSpeed(100)) {
                    startIntake();
                    shootTimer.resetTimer();
                    setPathState(13);
                }
                break;

            case 13:
                flywheel.loop(true);
                if (shootTimer.getElapsedTimeSeconds() > 3.0) {
                    stopIntake();
                    stopFlywheelAndRelease();
                    follower.followPath(shoot_done, true);
                    setPathState(99);
                }
                break;

            case 99:
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        shootTimer = new Timer();
        waitTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        intake1 = hardwareMap.get(DcMotorEx.class, "int1");
        intake2 = hardwareMap.get(DcMotorEx.class, "int2");
        outtake1 = hardwareMap.get(DcMotorEx.class, "l1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "l2");
        barrier1 = hardwareMap.get(Servo.class, "b1");
        barrier2 = hardwareMap.get(Servo.class, "b2");

        intake1.setDirection(DcMotorEx.Direction.FORWARD);
        intake2.setDirection(DcMotorEx.Direction.FORWARD);

        outtake1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rampServo = hardwareMap.get(Servo.class, "rampServo");

        flywheel = new FlywheelPIDTest();
        flywheel.init(hardwareMap, telemetry);

        closeBarriers();

        waitForStart();
        setPathState(0);

        while (opModeIsActive()) {
            rampServo.setPosition(0);
            follower.update();
            autonomousPathUpdate();

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("shoot timer", shootTimer.getElapsedTimeSeconds());
            telemetry.addData("flywheel at speed", flywheel.isAtSpeed(100));
            telemetry.update();
        }
    }

    public void startIntake() {
        intake1.setPower(1);
        intake2.setPower(1);
    }

    public void startRowIntake() {
        intake1.setPower(1);
        intake2.setPower(1);
        openBarriers();
    }

    public void stopIntake() {
        intake1.setPower(0);
        intake2.setPower(0);
    }

    public void stopRowIntake() {
        intake1.setPower(0);
        intake2.setPower(0);
        closeBarriers();
    }

    public void openBarriers() {
        barrier2.setPosition(0.80);
    }

    public void closeBarriers() {
        barrier2.setPosition(0.5);
    }

    public void stopFlywheelAndRelease() {
        flywheel.loop(false);
        outtake1.setPower(0);
        outtake2.setPower(0);
    }
}