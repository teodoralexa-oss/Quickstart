package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class SampleAuton extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Pose startPose = new Pose(26, 130, Math.toRadians(144));
    private Pose shootPose = new Pose(53, 85, Math.toRadians(130));
    private Pose rowPose = new Pose(15, 84, Math.toRadians(180));

    private Path start_shoot, shoot_pickup, pickup_shoot;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        start_shoot = new Path(new BezierLine(startPose, shootPose));
        start_shoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());
        shoot_pickup = new Path(new BezierLine(shootPose, rowPose));
        shoot_pickup.setConstantHeadingInterpolation(rowPose.getHeading());//turn first
        pickup_shoot = new Path(new BezierLine(rowPose, shootPose));
        pickup_shoot.setLinearHeadingInterpolation(rowPose.getHeading(), shootPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                follower.followPath(start_shoot,true);
                setPathState(1);}
                break;
            case 1:
                if(!follower.isBusy()) {
                follower.followPath(shoot_pickup);
                setPathState(99);}
                break;
            case 99:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        waitForStart();
        opmodeTimer.resetTimer();
        setPathState(0);
        while (opModeIsActive()) {
            // These loop the movements of the robot, these must be called continuously in order to work
            follower.update();
            autonomousPathUpdate();
            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}