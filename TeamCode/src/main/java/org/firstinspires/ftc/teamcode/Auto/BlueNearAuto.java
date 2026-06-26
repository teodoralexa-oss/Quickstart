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
public class BlueNearAuto extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, shootTimer, waitTimer, turnTimer;
    private int pathState;

    private Pose startPose = new Pose(26, 130, Math.toRadians(144));
    private Pose shootPose = new Pose(58, 87, Math.toRadians(134.7));
    private Pose turnPose = new Pose(59, 83, Math.toRadians(130));
    private Pose approachRow = new Pose(35, 92, Math.toRadians(0));
    private Pose rowPose = new Pose(26, 92, Math.toRadians(0));
    private Pose stopPose = new Pose(52, 61, Math.toRadians(180));
    private Pose approachRow2 = new Pose(35, 58, Math.toRadians(0));
    private Pose rowPose2 = new Pose(23, 58, Math.toRadians(0));
    private Pose stopPose2 = new Pose(22, 57, Math.toRadians(0));

    private DcMotorEx intake1;
    private DcMotorEx intake2;
    private DcMotorEx outtake1;
    private DcMotorEx outtake2;
    private Servo rampServo;
    private Servo barrier1;
    private Servo barrier2;
    private FlywheelPIDTest flywheel;

    private Path start_shoot, turn_to_intake, approach_row, go_to_row, pickup_shoot, shoot_stop, approach_row2, go_to_row2, pickup_shoot2, done_pose;

    public void buildPaths() {
        start_shoot = new Path(new BezierLine(startPose, shootPose));
        start_shoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        turn_to_intake = new Path(new BezierLine(shootPose, turnPose));
        turn_to_intake.setLinearHeadingInterpolation(shootPose.getHeading(), turnPose.getHeading());

        approach_row = new Path(new BezierLine(turnPose, approachRow));
        approach_row.setConstantHeadingInterpolation(approachRow.getHeading());

        go_to_row = new Path(new BezierLine(approachRow, rowPose));
        go_to_row.setConstantHeadingInterpolation(rowPose.getHeading());

        pickup_shoot = new Path(new BezierLine(rowPose, shootPose));
        pickup_shoot.setLinearHeadingInterpolation(rowPose.getHeading(), shootPose.getHeading());

        shoot_stop = new Path(new BezierLine(shootPose, stopPose));
        shoot_stop.setLinearHeadingInterpolation(shootPose.getHeading(), stopPose.getHeading());

        approach_row2 = new Path(new BezierLine(stopPose, approachRow2));
        approach_row2.setConstantHeadingInterpolation(approachRow2.getHeading());

        go_to_row2 = new Path(new BezierLine(approachRow2, rowPose2));
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
                    follower.setMaxPower(0.8);
                    follower.followPath(approach_row, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    startRowIntake();
                    follower.setMaxPower(0.4);
                    follower.followPath(go_to_row, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    stopRowIntake();
                    follower.setMaxPower(1.0);
                    follower.followPath(pickup_shoot, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    flywheel.targetRPM = 1750;
                    flywheel.loop(true);
                    waitTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8:
                flywheel.loop(true);
                if (waitTimer.getElapsedTimeSeconds() > 1.0 && flywheel.isAtSpeed(100)) {
                    startIntake();
                    shootTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9:
                flywheel.loop(true);
                if (shootTimer.getElapsedTimeSeconds() > 1.5) {
                    stopIntake();
                    stopFlywheelAndRelease();
                    follower.followPath(shoot_stop, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(approach_row2, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    startRowIntake();
                    follower.setMaxPower(0.4);
                    follower.followPath(go_to_row2, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    stopRowIntake();
                    follower.setMaxPower(1.0);
                    follower.followPath(pickup_shoot2, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    flywheel.targetRPM = 1750;
                    flywheel.loop(true);
                    waitTimer.resetTimer();
                    setPathState(14);
                }
                break;

            case 14:
                flywheel.loop(true);
                if (waitTimer.getElapsedTimeSeconds() > 1.0 && flywheel.isAtSpeed(100)) {
                    startIntake();
                    shootTimer.resetTimer();
                    setPathState(15);
                }
                break;

            case 15:
                flywheel.loop(true);
                if (shootTimer.getElapsedTimeSeconds() > 3.0) {
                    stopIntake();
                    stopFlywheelAndRelease();
                    setPathState(16);
                }
                break;

            case 16:
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