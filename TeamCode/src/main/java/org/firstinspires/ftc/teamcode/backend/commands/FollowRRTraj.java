package org.firstinspires.ftc.teamcode.backend.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.backend.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.backend.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.backend.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.backend.utilities.GamepadWrapper;

public class FollowRRTraj extends CommandBase {

    private final DrivetrainSubsystem dt;
    private final SampleMecanumDrive drive;
    private final TrajectorySequence traj;

    public FollowRRTraj(DrivetrainSubsystem dt, SampleMecanumDrive drive, TrajectorySequence traj) {
        this.dt = dt;
        addRequirements(dt);
        this.drive = drive;
        this.traj = traj;
    }

    @Override
    public void initialize() {drive.followTrajectorySequenceAsync(traj);}

    @Override
    public void execute() {drive.update();}

    @Override
    public boolean isFinished() { return !drive.isBusy(); }

    @Override
    public void end(boolean isInterrupted) {}
}
