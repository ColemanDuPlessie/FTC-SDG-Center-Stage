package org.firstinspires.ftc.teamcode.backend.cv;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.cv.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;


public class AprilTagEnabledLocalizer implements Localizer {

    private final MecanumDrive.MecanumLocalizer driveEncoders;
    private AprilTagDetectionPipeline aprilTagLocalizer;

    private HashMap<Integer, Pose2d> lastTagPositions = new HashMap<Integer, Pose2d>(); // Key is tag id, value is the estimated (absolute) pose of the April tag.
    private HashMap<Integer, Integer> lastTagDetectionTimes = new HashMap<Integer, Integer>(); // Key is tag id, value is time (in milliseconds) since last detected.
    private final int favoriteTag = 19; // The tag we trust the most. It overrides all other tags.

    public static double acceptableDPS = 0.5; // Stands for "acceptable drift per second," and is measured in in/sec. It's probably set way too high right now.
    public static double acceptableTotalDrift = 6.0; // The maximum that our pose estimate can drift before we believe that somebody moved our April tag.

    // Lens intrinsics
    // UNITS ARE PIXELS
    private double fx = 822.317; // 670.779069; // 1078.03779?
    private double fy = 822.317; // 674.806148; // 1084.50988?
    private double cx = 319.495; // 361.418117; // 580.850545?
    private double cy = 242.502; // 153.041358; // 245.959325?

    // DO NOT TOUCH THE COMMENT BELOW THIS ONE OR WE WON'T BE ABLE TO RE-CALIBRATE THE WEBCAM

    // <!-- Logitech HD Webcam C270, Calibrated by REDACTED, YYYY.MM.DD using 3DF Zephyr -->
    // <!--<Camera vid="Logitech" pid="0x0825">
    //     <Calibration
    // size="640 480"
    // focalLength="822.317f, 822.317f"
    // principalPoint="319.495f, 242.502f"
    // distortionCoefficients="-0.0449369, 1.17277, 0, 0, -3.63244, 0, 0, 0"
    //     />
    // </Camera> -->

    // UNITS ARE METERS
    private double tagsize = 0.163; // 0.166?

    public final double INCHES_PER_METER = 39.3701;

    private Pose2d poseEstimate = new Pose2d();
    private Pose2d cameraOffset;
    private ElapsedTime elapsedTime;

    public AprilTagEnabledLocalizer(@NonNull MecanumDrive drive, OpenCvCamera camera, Pose2d cameraOffset) {
        this.cameraOffset = cameraOffset;
        elapsedTime = new ElapsedTime();
        driveEncoders = new MecanumDrive.MecanumLocalizer(drive);
        initCamera(camera);
    }

    public AprilTagEnabledLocalizer(@NonNull MecanumDrive drive, OpenCvCamera camera, Pose2d cameraOffset, boolean useExternalHeading) {
        this.cameraOffset = cameraOffset;
        elapsedTime = new ElapsedTime();
        driveEncoders = new MecanumDrive.MecanumLocalizer(drive, useExternalHeading);
        initCamera(camera);
    }

    private void initCamera(OpenCvCamera camera) {
        aprilTagLocalizer = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagLocalizer);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    private Pose2d aprilTagPoseToPose2d(AprilTagPose pose) {
        return new Pose2d(new Vector2d(pose.x/INCHES_PER_METER, pose.y/INCHES_PER_METER), pose.yaw); // TODO this may need pose.z instead of pose.y
    }

    private Pose2d getAprilTagAbsolutePose(Pose2d robotPose, Pose2d aprilTagPose) {
        double robotHeading = robotPose.getHeading();
        Vector2d aprilTagRelativePosition = new Vector2d(aprilTagPose.getX(), aprilTagPose.getY());
        aprilTagRelativePosition = aprilTagRelativePosition.rotated(robotHeading);
        Pose2d aprilTagTruePose = new Pose2d(aprilTagRelativePosition, aprilTagPose.getHeading()).plus(robotPose);
        return aprilTagTruePose;
    }

    private Pose2d getRobotAbsolutePose(Pose2d absoluteTagPose, Pose2d relativeTagPose) {
        return getAprilTagAbsolutePose(absoluteTagPose, relativeTagPose.unaryMinus());
    }

    private Pose2d getCameraEstimatedPosition(Pose2d estimatedPose) {
        /**
         * Returns a Pose2d if the camera successfully detected a pose correction. Returns null if
         * the camera cannot see any April tags or if it believes the April tags that it has seen
         * have been moved and need to be re-calibrated.
         */
        ArrayList<AprilTagDetection> detections = aprilTagLocalizer.getDetectionsUpdate();
        if (detections == null || detections.size() == 0) { return null; } // There are no detected tags or there is no new frame
        HashMap<Integer, Pose2d> estimatedPoses = new HashMap<Integer, Pose2d>();
        for (AprilTagDetection detection : detections) { // For each tag we currently see
            if (!lastTagPositions.containsKey(detection)) { // If this tag has never been seen before
                Pose2d tagPose = getAprilTagAbsolutePose(estimatedPose, aprilTagPoseToPose2d(detection.pose)); // Figure out where we think the tag is
                lastTagPositions.put(detection.id, tagPose);
            } else { // If this tag has been seen before
                Pose2d robotPose = getRobotAbsolutePose(lastTagPositions.get(detection.id), aprilTagPoseToPose2d(detection.pose));
                double poseError = robotPose.minus(estimatedPose).vec().norm();
                double maxAcceptableError = Math.min(acceptableTotalDrift, acceptableDPS * (elapsedTime.milliseconds()-lastTagDetectionTimes.get(detection.id)) / 1000);
                if (poseError <= maxAcceptableError) { // If we think that error is due to odo drift
                    estimatedPoses.put(detection.id, robotPose); // Record where the tag claims the robot is
                } else { // If we think that error is due to the tag being moved
                    Pose2d tagPose = getAprilTagAbsolutePose(estimatedPose, aprilTagPoseToPose2d(detection.pose)); // Throw out our previous estimates and figure out where we think the tag is from scratch
                    lastTagPositions.put(detection.id, tagPose);
                }
            }
            lastTagDetectionTimes.put(detection.id, (int)elapsedTime.milliseconds()); // Record that we have detected a tag
        }
        if (estimatedPoses.size() == 0) { return null; } // We have found no useful detections
        else if (estimatedPoses.containsKey(favoriteTag)) {return estimatedPoses.get(favoriteTag);} // We have detected our favorite April tag
        return null; // TODO
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return driveEncoders.getPoseEstimate();
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        poseEstimate = pose2d;
        driveEncoders.setPoseEstimate(pose2d);
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return driveEncoders.getPoseVelocity();
    }

    @Override
    public void update() {
        driveEncoders.update();
        poseEstimate = driveEncoders.getPoseEstimate();
        Pose2d cameraEstimate = getCameraEstimatedPosition(poseEstimate);
        if (cameraEstimate != null) {
            setPoseEstimate(cameraEstimate);
        }
    }
}

