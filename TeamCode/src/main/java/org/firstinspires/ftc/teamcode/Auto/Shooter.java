package org.firstinspires.ftc.teamcode.Auto;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.teamcode.VroomVroomManual;
import org.firstinspires.ftc.teamcode.localization.SimplePedroDrive;
import org.firstinspires.ftc.teamcode.localization.TurretSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class Shooter extends LinearOpMode {

    SimplePedroDrive drive;
    TurretSubsystem turret;

    @Override
    public void runOpMode() {

        drive = new SimplePedroDrive(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            drive.update();

            Pose pose = drive.getPose();

            turret.update(pose);

            if (gamepad1.x) turret.setBlueGoal();
            if (gamepad1.b) turret.setRedGoal();

            telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
            telemetry.addData("Turret ticks", turret.turret.getCurrentPosition());
            telemetry.update();
        }
    }
}

