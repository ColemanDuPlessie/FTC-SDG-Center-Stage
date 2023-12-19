package org.firstinspires.ftc.teamcode.backend.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.backend.cv.TeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class CameraSubsystem extends SubsystemBase {

    private AprilTagProcessor aprilTag;
    private TeamPropProcessor propProcessor; // TODO this definitely isn't really an int
    private VisionPortal visionPortal;

    public enum PROP_POSITION {
        LEFT,
        CENTER,
        RIGHT
    }

    boolean teleop;

    public void init(HardwareMap ahwMap, boolean isTeleop) {
        teleop = isTeleop;
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
        builder.setCamera(ahwMap.get(WebcamName.class, "Camera"));
        builder.setCameraResolution(new Size(320, 240));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2); // Alternative is MJPEG
        builder.setAutoStopLiveView(true);
        builder.addProcessor(aprilTag);

        if (!isTeleop) {
            propProcessor = new TeamPropProcessor(); // TODO
            builder.addProcessor(propProcessor);
        }

        visionPortal = builder.build();

        if (isTeleop) {
            visionPortal.setProcessorEnabled(aprilTag, true);
        } else {
            visionPortal.setProcessorEnabled(aprilTag, false);
            visionPortal.setProcessorEnabled(propProcessor, true);
        }

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
                // detection.id;
                // detection.metadata.name;
                // detection.ftcPose
            }
        }
        return null;
    }

    public void stopStream() {visionPortal.stopStreaming();}
    public void startStream() {visionPortal.resumeStreaming();}
    public void stopATag() {visionPortal.setProcessorEnabled(aprilTag, false);}
    public void startATag() {visionPortal.setProcessorEnabled(aprilTag, true);}
    public void killCamera() {visionPortal.close();}

    public void propDetected() {
        visionPortal.setProcessorEnabled(propProcessor, false);
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    @Override
    public void periodic() {
    }

}
