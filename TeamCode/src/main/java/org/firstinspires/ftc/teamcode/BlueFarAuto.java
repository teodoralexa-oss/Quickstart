package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
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
    private final Pose shootPose = new Pose(83, 14, Math.toRadians(130));
    private PathChain driveStartPosShootPos;
    public void buildPaths(){
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
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
