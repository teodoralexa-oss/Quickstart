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
public class RedNearAuto extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, shootTimer, waitTimer, turnTimer;
    private int pathState;

    private Pose startPose = new Pose(117, 130, Math.toRadians(35));
    private Pose shootPose = new Pose(80, 82, Math.toRadians(49));
    private Pose turnPose = new Pose(83, 79.5, Math.toRadians(0));
    private Pose rowPose = new Pose(120, 77.5, Math.toRadians(180));
    private Pose stopPose = new Pose(82, 55, Math.toRadians(0));
    private Pose rowPose2 = new Pose(120, 55, Math.toRadians(180));
    private Pose stopPose2 = new Pose(120, 53, Math.toRadians(180));
    private Pose barPose = new Pose(19, 59, Math.toRadians(0));

    private DcMotorEx intake1;
    private DcMotorEx intake2;
    private DcMotorEx outtake1;
    private DcMotorEx outtake2;
    private Servo rampServo;
    private Servo barrier1;
    private Servo barrier2;
    private FlywheelPIDTest flywheel;

    private Path start_shoot, turn_to_intake, go_to_row, pickup_shoot, shoot_stop, go_to_row2, pickup_shoot2,done_pose;

    double reverseOuttakeRPM = 500;

    public void buildPaths() {
        start_shoot = new Path(new BezierLine(startPose, shootPose));
        start_shoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        turn_to_intake = new Path(new BezierLine(shootPose, turnPose));
        turn_to_intake.setLinearHeadingInterpolation(shootPose.getHeading(), turnPose.getHeading());

        go_to_row = new Path(new BezierLine(turnPose, rowPose));
        go_to_row.setConstantHeadingInterpolation(rowPose.getHeading());

        pickup_shoot = new Path(new BezierLine(rowPose, shootPose));
        pickup_shoot.setLinearHeadingInterpolation(rowPose.getHeading(), shootPose.getHeading());

        shoot_stop = new Path(new BezierLine(shootPose, stopPose));
        shoot_stop.setLinearHeadingInterpolation(shootPose.getHeading(), stopPose.getHeading());

        go_to_row2 = new Path(new BezierLine(stopPose, rowPose2));
        go_to_row2.setConstantHeadingInterpolation(rowPose2.getHeading());

        pickup_shoot2 = new Path(new BezierLine(rowPose2, shootPose));
        pickup_shoot2.setLinearHeadingInterpolation(rowPose2.getHeading(), shootPose.getHeading());

        done_pose = new Path(new BezierLine(shootPose, stopPose2));
        done_pose.setLinearHeadingInterpolation(shootPose.getHeading(), stopPose2.getHeading());
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
                    flywheel.targetRPM = 1710;
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
                if (shootTimer.getElapsedTimeSeconds() > 1.5) {
                    stopIntake();
                    stopFlywheelAndRelease();
                    follower.followPath(turn_to_intake, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    startRowIntake();
                    follower.setMaxPower(0.8);
                    follower.followPath(go_to_row, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    stopRowIntake();
                    follower.setMaxPower(1.0);
                    follower.followPath(pickup_shoot, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    flywheel.targetRPM = 1750;
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
                if (shootTimer.getElapsedTimeSeconds() > 1.5) {
                    stopIntake();
                    stopFlywheelAndRelease();
                    follower.followPath(shoot_stop, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    turnTimer.resetTimer();
                    follower.setPose(stopPose);
                    setPathState(10);
                }
                break;

            case 10:
                if (turnTimer.getElapsedTimeSeconds() > 0.5) {
                    startRowIntake();
                    follower.setMaxPower(0.8);
                    follower.followPath(go_to_row2, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    stopRowIntake();
                    follower.setMaxPower(1.0);
                    follower.followPath(pickup_shoot2, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    flywheel.targetRPM = 1750;
                    flywheel.loop(true);
                    waitTimer.resetTimer();
                    setPathState(13);
                }
                break;

            case 13:
                flywheel.loop(true);
                if (waitTimer.getElapsedTimeSeconds() > 1.0 && flywheel.isAtSpeed(100)) {
                    startIntake();
                    shootTimer.resetTimer();
                    setPathState(14);
                }
                break;

            case 14:
                flywheel.loop(true);
                if (shootTimer.getElapsedTimeSeconds() > 3.0) {
                    stopIntake();
                    stopFlywheelAndRelease();
                    setPathState(15);
                }
                break;
            case 15:
                follower.followPath(done_pose, true);
                setPathState(99);
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
        turnTimer = new Timer();

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
        //barrier1.setPosition(0.20);
        barrier2.setPosition(0.80);
    }

    public void closeBarriers() {
        //barrier1.setPosition(0.5);
        barrier2.setPosition(0.5);
    }

    public void stopFlywheelAndRelease() {
        flywheel.loop(false);
        outtake1.setPower(0);
        outtake2.setPower(0);
    }
}