package org.firstinspires.ftc.teamcode.backend.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.IntakeSlidesSubsystem;

public class SmoothSetIntakeSlides extends CommandBase {

    private ElapsedTime timer;
    private long millis;
    private double targetPos;
    private double startPos;
    private long startMillis;
    private IntakeSlidesSubsystem slides;

    public SmoothSetIntakeSlides(IntakeSlidesSubsystem s, double targetPos, long millis, ElapsedTime timer) {
        slides = s;
        addRequirements(s);
        this.targetPos = targetPos;
        this.millis = millis;
        this.timer = timer;
    }

    @Override
    public void initialize() {
        this.startMillis = (long) timer.milliseconds();
        this.startPos = slides.getPosition();
    }

    @Override
    public void execute() {
        slides.setTargetPosition(startPos + (targetPos - startPos)*((long)timer.milliseconds()-startMillis)/millis);
    }

    @Override
    public boolean isFinished() {return timer.milliseconds() >= millis + startMillis;}

    @Override
    public void end(boolean interrupted) {
    }
}
