package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-43.687241447082236)
            .lateralZeroPowerAcceleration(-69.4953471105549)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06,0,0.0001,0.025))
            //.headingPIDFCoefficients(new PIDFCoefficients(0.43,0,0.002,0.43))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.6,0.0,0,0.6,0.025))
            .centripetalScaling(0.0005)
            .mass(6.5);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("dreaptaFataMotor")
            .rightRearMotorName("dreaptaSpateMotor")
            .leftRearMotorName("stangaSpateMotor")
            .leftFrontMotorName("stangaFataMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(9899.416700538508)
            .yVelocity(-1411.1643549301614);
    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1,
            1);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6)
            .strafePodX(4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
