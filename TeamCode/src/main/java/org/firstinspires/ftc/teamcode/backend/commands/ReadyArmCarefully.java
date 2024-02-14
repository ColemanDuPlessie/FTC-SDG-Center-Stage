package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.WristSubsystem;

@Config
public class ReadyArmCarefully extends CommandBase {

    public static long duration = 500;

    private ElapsedTime timer;
    private long startMillis;
    private ArmSubsystem arm;
    private WristSubsystem wrist;
    private double startWristPos;
    private double startArmPos;
    private boolean isLow;

    private double armTargetPos;

    public ReadyArmCarefully(ArmSubsystem a, WristSubsystem w, ElapsedTime timer) {
        arm = a;
        wrist = w;
        addRequirements(a);
        addRequirements(w);
        this.timer = timer;
        isLow = false;
    }

    public ReadyArmCarefully(ArmSubsystem a, WristSubsystem w, ElapsedTime timer, boolean isLow) {
        arm = a;
        wrist = w;
        addRequirements(a);
        addRequirements(w);
        this.timer = timer;
        this.isLow = isLow;
    }

    @Override
    public void initialize() {
        /**
         * Assumes that the arm and wrist are already in the pre-deposit position (deposit for
         * the arm and center for the wrist).
         */
        this.startMillis = (long) timer.milliseconds();
        startWristPos = wrist.getPosition();
        startArmPos = arm.getPosition();
        armTargetPos = isLow ? ArmSubsystem.upLowPosition : ArmSubsystem.upPosition;
        wrist.deposit();
    }

    @Override
    public void execute() {
        double completeness = (timer.milliseconds() - startMillis) / duration;
        completeness = Math.min(completeness, 1.0);
        wrist.setTargetPosition(WristSubsystem.readyPosition*completeness + startWristPos*(1-completeness));
        arm.setTargetPosition(armTargetPos*completeness + startArmPos*(1-completeness));
    }

    @Override
    public boolean isFinished() {return ((long) timer.milliseconds()) - startMillis >= duration;}

    @Override
    public void end(boolean interrupted) {
        wrist.ready();
        arm.deposit();
    }
}
