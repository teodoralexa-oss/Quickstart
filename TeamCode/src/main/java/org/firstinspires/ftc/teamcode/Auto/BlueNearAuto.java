package org.firstinspires.ftc.teamcode.Auto;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Auto.Shooter;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueNearAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState{
        //START
        //DRIVE
        //SHOOT
        DRIVE_STARTPOS_SHOOT_POS,
        DRIVE_BASE_PICKUP_POS,
        DRIVE_BACK_PICKUP_SHOOT,
        SHOOT_PRELOAD
    }
    PathState pathState;
    private final Pose startPose = new Pose(34, 136, Math.toRadians(90));
    private final Pose shootPose = new Pose(31, 119, Math.toRadians(-45));
    private final Pose middlePose = new Pose(58, 101.5, Math.toRadians(-90));
    private final Pose nearTrPose = new Pose(93, 11, Math.toRadians(-90));
    private final Pose basePose = new Pose(129, 11.5, Math.toRadians(-90));
    private PathChain driveStartPosShootPos;
    private PathChain driveBasePickupPose;
    private PathChain driveBackShoot;
    public DcMotorEx motor1;
    public DcMotorEx motor2;
    public void buildPaths(){
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
        driveBasePickupPose = follower.pathBuilder()
                .addPath(new BezierLine(middlePose,nearTrPose))
                .setLinearHeadingInterpolation(middlePose.getHeading(),nearTrPose.getHeading())
                .addPath(new BezierLine(basePose,basePose))
                .setLinearHeadingInterpolation(basePose.getHeading(), basePose.getHeading())
                .addPath(new BezierLine(nearTrPose,middlePose))
                .setLinearHeadingInterpolation(nearTrPose.getHeading(), middlePose.getHeading())
                .build();
        driveBackShoot = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,middlePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),middlePose.getHeading())
                .build();
    }
    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                //setPathState(PathState.SHOOT_PRELOAD);
                pathState=PathState.SHOOT_PRELOAD;
                break;
            case DRIVE_BASE_PICKUP_POS:
                follower.followPath(driveBasePickupPose, true);
                //setPathState(PathState.SHOOT_PRELOAD);
                //pathState=PathState.SHOOT_PRELOAD;
                break;
            case DRIVE_BACK_PICKUP_SHOOT:
                follower.followPath(driveBackShoot, true);
                //setPathState(PathState.SHOOT_PRELOAD);
                pathState=PathState.SHOOT_PRELOAD;
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    //SHOOT
                    Shoot();
                    telemetry.addLine("Done Path");
                }
                break;
            default:
                telemetry.addLine("No State");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = pathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        //opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        //Add any mechanics=)

        buildPaths();
        follower.setPose(startPose);
    }
    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
        motor1= hardwareMap.get(DcMotorEx.class, "l1");
        motor2 = hardwareMap.get(DcMotorEx.class, "l2");
    }
    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
    }
    public void Shoot(){
        while(true){
            motor1.setPower(-1.0);
            motor2.setPower(1.0);
            sleep(2000);
            break;
        }
        while (true){
            motor1.setPower(0.7);
            motor2.setPower(-0.7);
            sleep(2000);
            break;
        }
        motor1.setPower(0);
        motor2.setPower(0);
    }
}
