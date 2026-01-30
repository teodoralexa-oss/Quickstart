package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTag {
    private AprilTagProcessor apriltagprocessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detection = new ArrayList<>();
    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;
        apriltagprocessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
            .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "WebCam1"));
        builder.setCameraResolution(new Size(640,480));
        builder.addProcessor(apriltagprocessor);

        visionPortal = builder.build();
    }

    public void update(){
        detection = apriltagprocessor.getDetections();
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedid){
        if(detectedid == null){return;}
        if (detectedid.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedid.id, detectedid.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedid.ftcPose.x, detectedid.ftcPose.y, detectedid.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedid.ftcPose.pitch, detectedid.ftcPose.roll, detectedid.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedid.ftcPose.range, detectedid.ftcPose.bearing, detectedid.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedid.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedid.center.x, detectedid.center.y));
        }
    }
    public List<AprilTagDetection> getDetection() {
        return detection;
    }
    public AprilTagDetection getTagBySpecificid(int id){
        for(AprilTagDetection detection1 : detection){
            if(detection1.id == id){
                return detection1;
            }
        }
        return null;
    }
    public void stop(){
        if(visionPortal != null){
            visionPortal.close();
        }
    }
}
