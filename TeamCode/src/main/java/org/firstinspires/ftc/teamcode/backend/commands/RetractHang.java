package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;

@Config
public class RetractHang extends CommandBase {

    public static double finalPos = 0.2;
    public static long travelDuration = 2000;

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
        if (slides.getTargetPosition() > 1.02) { // If we are in hang position
            isRunning = true;
            startMillis = (long)timer.milliseconds();
            slides.setTargetPosition(0.99);
        } else { isRunning = false; }
    }

    @Override
    public void execute() {
        long totalElapsedTime = (long)timer.milliseconds()-startMillis;
        if (slides.getTargetPosition() > 1.02 || !isRunning) {
            isRunning = false;
            return;
        }
        double currentTarget = 0.99-((double)totalElapsedTime)/((double)travelDuration)*(0.99-finalPos);
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
