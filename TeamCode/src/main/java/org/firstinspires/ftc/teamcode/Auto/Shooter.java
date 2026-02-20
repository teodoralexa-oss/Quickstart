package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.localization.SimplePedroDrive;
import org.firstinspires.ftc.teamcode.localization.TurretSubsystem;

@TeleOp
public class Shooter extends LinearOpMode {
    SimplePedroDrive drive;
    TurretSubsystem turret;

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