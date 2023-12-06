package org.firstinspires.ftc.teamcode.backend.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.backend.cv.TeamShippingElementDetector;
import org.firstinspires.ftc.teamcode.backend.cv.pipelines.TeamPropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

@Config
public class CameraSubsystem extends SubsystemBase {

    public OpenCvCamera camera; // For team prop stuff
    public OpenCvPipeline pipeline;

    private AprilTagProcessor aprilTag; // For built in ATag stuff
    private VisionPortal visionPortal;

    public enum pipelineType {
        PROP_DETECTOR,
        BACKDROP_LOCALIZER
    }

    public enum PROP_POSITION {
        LEFT,
        CENTER,
        RIGHT
    }

    pipelineType livePipeline;

    public void init(HardwareMap ahwMap, pipelineType p) {
        livePipeline = p;
        if (p == pipelineType.BACKDROP_LOCALIZER) {
            initAprilTag(ahwMap);
        } else {
            initProp(ahwMap);
        }
    }

    private void initProp(HardwareMap h) {
        int cameraMonitorViewId = h.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", h.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(h.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        camera.setPipeline(new TeamPropDetectionPipeline(1, 822.317/2, 822.317/2, 319.495/2, 242.502/2));
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    private void initAprilTag(HardwareMap h) {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(822.317/2, 822.317/2, 319.495/2, 242.502/2) // TODO these parameters are fx, fy, cx, cy.
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3); // Lower decimation for higher detection range, but worse performance

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(h.get(WebcamName.class, "Camera"));
        builder.setCameraResolution(new Size(320, 240));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG); // Alternative is YUY2
        builder.setAutoStopLiveView(true);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        visionPortal.setProcessorEnabled(aprilTag, true);

    }

    public PROP_POSITION getPropPosition() {
        return PROP_POSITION.CENTER; // TODO
    }

    public Pose2d getBackdropPosition() { // TODO
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.size() == 0) {
            return null;
        }

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        return null;
    }

    public void stopStream() {
        if (livePipeline == pipelineType.BACKDROP_LOCALIZER) {
            visionPortal.stopStreaming();
        } else {
            return; // TODO
        }
    }

    public void startStream() {
        if (livePipeline == pipelineType.BACKDROP_LOCALIZER) {
            visionPortal.resumeStreaming();
        } else {
            return; // TODO
        }
    }

    public void killCamera() {
        if (livePipeline == pipelineType.BACKDROP_LOCALIZER) {
            visionPortal.close();
        } else {
            return; // TODO
        }
    }

    @Override
    public void periodic() {
    }

}
