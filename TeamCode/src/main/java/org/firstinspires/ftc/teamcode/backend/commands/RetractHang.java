package org.firstinspires.ftc.teamcode.backend.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;

public class RetractHang extends CommandBase {

    public static double finalPos = 0.4;
    public static long travelDuration = 3500;

    private SlidesSubsystem slides;
    private ElapsedTime timer;

    private boolean isRunning = false;
    private long startMillis;

    public RetractHang(SlidesSubsystem s, ElapsedTime timer) {
        slides = s;
        addRequirements(s);
        this.timer = timer;
    }

    @Override
    public void initialize() {
        if (slides.getTargetPosition() > 1.0) { // If we are in hang position
            isRunning = true;
            startMillis = (long)timer.milliseconds();
            slides.setTargetPosition(1.0);
        } else { isRunning = false; }
    }

    @Override
    public void execute() {
        long totalElapsedTime = (long)timer.milliseconds()-startMillis;
        if (slides.getTargetPosition() > 1.0 || !isRunning) {
            isRunning = false;
            return;
        }
        double currentTarget = 1.0-totalElapsedTime/travelDuration*(1-finalPos);
        if (currentTarget < finalPos) {
            slides.setTargetPosition(finalPos);
            isRunning = false;
        } else {
            slides.setTargetPosition(currentTarget);
        }
    }

    @Override
    public boolean isFinished() { return !isRunning; }
}
