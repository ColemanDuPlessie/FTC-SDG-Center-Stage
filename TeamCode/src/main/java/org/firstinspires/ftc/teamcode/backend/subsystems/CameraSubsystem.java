package org.firstinspires.ftc.teamcode.backend.subsystems;

import static java.lang.Thread.sleep;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SetDrivingStyle;
import org.firstinspires.ftc.teamcode.backend.cv.AprilTagProcessorWithDash;
import org.firstinspires.ftc.teamcode.backend.cv.TeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class CameraSubsystem extends SubsystemBase {

    private AprilTagProcessorWithDash aprilTag;
    private TeamPropProcessor propProcessor;
    private VisionPortal visionPortal;

    boolean teleop;

    public void init(HardwareMap ahwMap, boolean isTeleop) {
        teleop = isTeleop;
        aprilTag = new AprilTagProcessorWithDash.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502) // these parameters are fx, fy, cx, cy.
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
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2); // Alternative is MJPEG
        builder.setAutoStopLiveView(true);
        builder.addProcessor(aprilTag);

        if (!isTeleop) {
            propProcessor = new TeamPropProcessor(SetDrivingStyle.isBlue);
            builder.addProcessor(propProcessor);
        }

        visionPortal = builder.build();

        if (isTeleop) {
            visionPortal.setProcessorEnabled(aprilTag, true);
        } else {
            visionPortal.setProcessorEnabled(aprilTag, false);
            visionPortal.setProcessorEnabled(propProcessor, true);
            FtcDashboard.getInstance().startCameraStream(propProcessor, 0);
        }

/*
        try {
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            } // TODO uber janky
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        setManualExposure(6, 250); // TODO tune
*/
    }

    public TeamPropProcessor.PROP_POSITION getPropPosition() {
        return propProcessor.getCurrentPosition();
    }

    public double getPropConfidence() {
        return propProcessor.getCurrentConfidence();
    }

    public List<AprilTagDetection> getRawTagDetections() {
        return aprilTag.getDetections();
    }

    public Pose2d getBackdropPosition() { // TODO
        /**
         * Returns a pose2d whose origin is the center AprilTag and whose heading 0 is pointing
         * toward the backdrop. Note that we ignore a lot of heading info because it's unreliable
         */
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.size() == 0) {
            return null;
        }

        int[] targetIds;
        if (SetDrivingStyle.isBlue) {
            targetIds = new int[]{1, 2, 3};
        } else {
            targetIds = new int[]{4, 5, 6};
        }

        ArrayList<Pose2d> poseGuesses = new ArrayList<Pose2d>();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (Arrays.stream(targetIds).anyMatch(i -> i == detection.id)) {
                    Pose2d tempPose = new Pose2d(new Vector2d(detection.ftcPose.x, detection.ftcPose.y), -detection.ftcPose.yaw); // There was previously a .rotated(yaw) after the vector definition, but it seemed to negatively impact performance.
                    if (detection.id % 3 == 1) {
                        tempPose.minus(new Pose2d(6, 0, 0));
                    } else if (detection.id % 3 == 0) {
                        tempPose.plus(new Pose2d(6, 0, 0));
                    }
                    poseGuesses.add(tempPose);
                }
            }
        }
        if (poseGuesses.size() == 0) {return null;}
        double bestGuessX = 0;
        double bestGuessY = 0;
        double bestGuessHeading = 0;
        for (Pose2d toAdd:poseGuesses) {
            bestGuessX += toAdd.getX();
            bestGuessY += toAdd.getY();
            bestGuessHeading += toAdd.getHeading();
        }
        return new Pose2d(bestGuessX, bestGuessY, bestGuessHeading);
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

    public void setManualExposure(int exposureMS, int gain) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
    }

}
