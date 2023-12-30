package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.backend.utilities.GamepadWrapper;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.PIDController;

public class AutoTargetBackdrop extends CommandBase {

    private final DrivetrainSubsystem dt;
    private final CameraSubsystem camera;
    private final SlidesSubsystem slides;
    private final GamepadWrapper gamepad;

    public static double kPf = 0.005;
    public static double kIf = 0.000;
    public static double kDf = 0.015;
    public static double kPt = 0.01;
    public static double kIt = 0.0001;
    public static double kDt = 0.03;

    private final PIDController forwardPID;
    private final PIDController turnPID;

    public static double speed = 0.35;
    public static double minYDist = 3;
    public static double maxYDist = 12;

    public AutoTargetBackdrop(DrivetrainSubsystem dt, CameraSubsystem camera, GamepadWrapper gamepad, ElapsedTime aTimer, SlidesSubsystem slides) {
        this.gamepad = gamepad;
        this.dt = dt;
        this.camera = camera;
        addRequirements(dt);
        addRequirements(camera);
        forwardPID = new PIDController(kPf, kIf, kDf, aTimer);
        turnPID = new PIDController(kPt, kIt, kDt, aTimer);
        this.slides = slides;
    }

    @Override
    public void initialize() {camera.startATag();}

    @Override
    public void execute() {
        double targetYDist = slides.getPosition()*(maxYDist-minYDist)+maxYDist;
        Pose2d currentPose = camera.getBackdropPosition();
        double forward, turn;
        if (currentPose == null) {
            forward = 0.5;
            turn = 0.0;
        } else {
            forward = forwardPID.update(currentPose.getY(), targetYDist);
            turn = -turnPID.update(currentPose.getHeading(), 0.0);
        }
        double strafe = gamepad.getLeftStickX()*0.5;
        dt.driveSimple(forward, turn, strafe, speed);
    }

    @Override
    public boolean isFinished() { return !gamepad.getY(); }

    @Override
    public void end(boolean isInterrupted) {camera.stopATag();}
}
