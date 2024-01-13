package org.firstinspires.ftc.teamcode.backend.roadrunner.bespoke;


import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.backend.subsystems.CameraSubsystem;

public class MecanumLocalizerWithATag implements Localizer {

    private final MecanumDrive.MecanumLocalizer driveEncoders;
    private final CameraSubsystem camera;

    public MecanumLocalizerWithATag(MecanumDrive drive, CameraSubsystem camera) {
        driveEncoders = new MecanumDrive.MecanumLocalizer(drive);
        this.camera = camera;
    }

    @Override
    public Pose2d getPoseEstimate() {
        return driveEncoders.getPoseEstimate();
    }

    @Override
    public void setPoseEstimate(Pose2d pose2d) {
        driveEncoders.setPoseEstimate(pose2d);
    }

    @Override
    public Pose2d getPoseVelocity() {
        return driveEncoders.getPoseVelocity();
    }

    @Override
    public void update() {
        driveEncoders.update();
        // TODO actually implement AprilTag stuff
    }
}
