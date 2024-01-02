package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.WristSubsystem;

@Config
public class ReadyArmCarefully extends CommandBase {

    public static long duration = 750;

    private ElapsedTime timer;
    private long startMillis;
    private ArmSubsystem arm;
    private WristSubsystem wrist;
    private double startWristPos;
    private double startArmPos;

    public ReadyArmCarefully(ArmSubsystem a, WristSubsystem w, ElapsedTime timer) {
        arm = a;
        wrist = w;
        addRequirements(a);
        addRequirements(w);
        this.timer = timer;
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
        wrist.deposit();
    }

    @Override
    public void execute() {
        double completeness = (timer.milliseconds() - startMillis) / duration;
        completeness = Math.min(completeness, 1.0);
        wrist.setTargetPosition(WristSubsystem.readyPosition*completeness + startWristPos*(1-completeness));
        arm.setTargetPosition(ArmSubsystem.upPosition*completeness + startArmPos*(1-completeness));
    }

    @Override
    public boolean isFinished() {return ((long) timer.milliseconds()) - startMillis >= duration;}

    @Override
    public void end(boolean interrupted) {
        wrist.ready();
        arm.deposit();
    }
}