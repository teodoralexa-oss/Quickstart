package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueFarAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState{
        //START
        //DRIVE
        //SHOOT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD
    }
    PathState pathState;
    private final Pose startPose = new Pose(83, 9, Math.toRadians(90));
    private final Pose shootPose = new Pose(83, 14, Math.toRadians(127));
    private final Pose trianglePose = new Pose(83, 9, Math.toRadians(90));
    private final Pose basePose = new Pose(123, 13, Math.toRadians(90));
    private final Pose trianglePose2 = new Pose(83, 9, Math.toRadians(90));
    private final Pose straightPose = new Pose(83, 33, Math.toRadians(90));
    private final Pose pollintake1 = new Pose(121, 33, Math.toRadians(180));
    private PathChain driveStartPosShootPos;
    private PathChain backBasePos;
    public void buildPaths(){
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .addPath(new BezierLine(trianglePose,basePose))
                .setLinearHeadingInterpolation(trianglePose.getHeading(),basePose.getHeading())
                .addPath(new BezierLine(trianglePose2,straightPose))
                .setLinearHeadingInterpolation(trianglePose2.getHeading(),straightPose.getHeading()).addPath(new BezierLine(trianglePose2,straightPose))
                .setLinearHeadingInterpolation(pollintake1.getHeading(),trianglePose2.getHeading())
                .addPath(new BezierLine(pollintake1,trianglePose2))

                .build();
    }
    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    //SHOOT
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
    }
    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
    }
}
