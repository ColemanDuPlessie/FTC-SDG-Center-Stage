package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;

@Config
public class ArmAwareSetSlides extends CommandBase {

    public static double changeoverPosition = 0.25;
    public static long armTravelWaitTime = 250;

    private ElapsedTime timer;
    private double targetPos;
    private double startPos;
    private long startMillis;
    private SlidesSubsystem slides;
    private ArmSubsystem arm;

    private boolean waitToLower = false;
    private boolean waitToRaise = false;

    public ArmAwareSetSlides(SlidesSubsystem s, ArmSubsystem a, double targetPos, ElapsedTime timer) {
        slides = s;
        arm = a;
        addRequirements(s);
        addRequirements(a);
        this.targetPos = targetPos;
        this.timer = timer;
    }

    @Override
    public void initialize() {
        this.startMillis = (long) timer.milliseconds();
        double startPos = slides.getPosition();
        if (startPos < changeoverPosition && targetPos > changeoverPosition) {
            arm.holding();
            waitToRaise = true;
            slides.setTargetPosition(targetPos);
        } else if (startPos > changeoverPosition && targetPos < changeoverPosition) {
            arm.holding();
            waitToLower = true;
        } else if (targetPos < changeoverPosition) {
            arm.holding();
            slides.setTargetPosition(targetPos);
        } else {
            arm.center();
            slides.setTargetPosition(targetPos);
        }
    }

    @Override
    public void execute() {
        if (waitToLower && ((long) timer.milliseconds()) - startMillis >= armTravelWaitTime) {
            slides.setTargetPosition(targetPos);
            waitToLower = false;
        }
        if (waitToRaise && slides.getPosition() >= changeoverPosition) {
            arm.center();
            waitToRaise = false;
        }
    }

    @Override
    public boolean isFinished() {return !(waitToLower || waitToRaise);}

    @Override
    public void end(boolean interrupted) {
    }
}