package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.backend.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.backend.utilities.GamepadWrapper;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.PIDController;

@Config
public class AutoTargetBackdrop extends CommandBase {

    private final DrivetrainSubsystem dt;
    private final CameraSubsystem camera;
    private final SlidesSubsystem slides;
    private final GamepadWrapper gamepad;

    public static double kPf = 0.5;
    public static double kIf = 0.000;
    public static double kDf = 1.0;
    public static double kPt = 0.7;
    public static double kIt = 0.03;
    public static double kDt = 1.4;

    private final PIDController forwardPID;
    private final PIDController turnPID;

    private double forward;
    private double turn;
    private Pose2d currentPose;


    public static double speed = 0.25;
    public static double minYDist = 4.5;
    public static double maxYDist = 12.5;

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
        currentPose = camera.getBackdropPosition();
        if (currentPose == null) { // If we have no detections, use slow, careful manual control
            forward = gamepad.getLeftStickY()*-0.25;
            turn = gamepad.getRightStickX()*0.5;
        } else { // If we have at least one detection, use it instead
            forward = forwardPID.update(currentPose.getY(), targetYDist);
            turn = -turnPID.update(currentPose.getHeading(), 0.0);
        }
        double strafe = gamepad.getLeftStickX()*0.5; // Always strafe manually
        dt.driveSimple(forward, turn, strafe, speed);
    }

    @Override
    public boolean isFinished() { return !gamepad.getY(); }

    @Override
    public void end(boolean isInterrupted) {camera.stopATag();}

    public void debug(Telemetry t) {
        if (currentPose != null) {
            t.addData("X dist", currentPose.getX());
            t.addData("Y dist", currentPose.getY());
            t.addData("Heading", currentPose.getHeading());
            t.addLine();
            t.addData("Y spd", forward);
            t.addData("turn spd", turn);
        }
    }
}
