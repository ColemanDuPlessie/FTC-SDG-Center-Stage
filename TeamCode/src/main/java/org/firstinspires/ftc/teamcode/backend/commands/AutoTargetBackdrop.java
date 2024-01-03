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

    public static double kPf = 0.2;
    public static double kIf = 0.000;
    public static double kDf = 0.4;

    private final PIDController forwardPID;

    private double forward;
    private double turn;
    private double targetYDist;
    private Pose2d currentPose;
    private Pose2d rollingAverageTruePose = new Pose2d(0, 0, 0);
    public static double truePoseDecay = 0.75;

    public static double speed = 0.225;
    public static double minYDist = 4.5;
    public static double maxYDist = 12.5;

    public AutoTargetBackdrop(DrivetrainSubsystem dt, CameraSubsystem camera, GamepadWrapper gamepad, ElapsedTime aTimer, SlidesSubsystem slides) {
        this.gamepad = gamepad;
        this.dt = dt;
        this.camera = camera;
        addRequirements(dt);
        addRequirements(camera);
        forwardPID = new PIDController(kPf, kIf, kDf, aTimer);
        this.slides = slides;
    }

    @Override
    public void initialize() {camera.startATag();}

    @Override
    public void execute() {
        targetYDist = slides.getPosition()*(maxYDist-minYDist)+minYDist;
        currentPose = camera.getBackdropPosition();
        if (currentPose == null) { // If we have no detections, use slow, careful manual control
            forward = gamepad.getLeftStickY()*0.25;

        } else { // If we have at least one detection, use it instead
            rollingAverageTruePose = rollingAverageTruePose.times(truePoseDecay).plus(currentPose.times(1.0-truePoseDecay));
            forward = forwardPID.update(rollingAverageTruePose.getY(), targetYDist);
        }
        turn = gamepad.getRightStickX()*0.5;
        double strafe = gamepad.getLeftStickX()*0.5; // Always strafe and turn manually
        dt.driveSimple(forward, turn, strafe, speed);
    }

    @Override
    public boolean isFinished() { return !gamepad.getY(); }

    @Override
    public void end(boolean isInterrupted) {} // TODO camera.stopATag();

    public void debug(Telemetry t) {
        if (currentPose != null) {
            t.addData("X dist", rollingAverageTruePose.getX());
            t.addData("Y dist", rollingAverageTruePose.getY());
            t.addData("Target Y dist", targetYDist);
            t.addData("Heading", rollingAverageTruePose.getHeading());
            t.addLine();
            t.addData("Y spd", forward);
            t.addData("turn spd", turn);
            t.addLine();
        }
    }
}
