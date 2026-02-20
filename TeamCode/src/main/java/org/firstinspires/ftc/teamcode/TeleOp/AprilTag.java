package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTag {

    private AprilTagProcessor processor;
    private VisionPortal visionPortal;
    private Telemetry telemetry;
    private boolean cameraOK = false;
    private boolean exposureSet = false;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        try {
            processor = new AprilTagProcessor.Builder()
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                    .build();

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hwMap.get(WebcamName.class, "WebCam1"))
                    .setCameraResolution(new Size(640, 480))
                    .enableLiveView(false)
                    .addProcessor(processor)
                    .build();

            cameraOK = true;
        } catch (Exception e) {
            telemetry.addLine("APRILTAG: CAMERA NOT FOUND");
            cameraOK = false;
        }
    }

    public void update() {
        if (!cameraOK || visionPortal == null) return;

        if (!exposureSet &&
                visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {

            try {
                ExposureControl exposure =
                        visionPortal.getCameraControl(ExposureControl.class);
                GainControl gain =
                        visionPortal.getCameraControl(GainControl.class);

                exposure.setMode(ExposureControl.Mode.Manual);
                exposure.setExposure(6, TimeUnit.MILLISECONDS);
                gain.setGain(240);

                exposureSet = true;
            } catch (Exception ignored) {}
        }
    }

    public AprilTagDetection getTagBySpecificid(int id) {
        if (!cameraOK || processor == null) return null;

        List<AprilTagDetection> detections = processor.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.id == id) return d;
        }
        return null;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
}
