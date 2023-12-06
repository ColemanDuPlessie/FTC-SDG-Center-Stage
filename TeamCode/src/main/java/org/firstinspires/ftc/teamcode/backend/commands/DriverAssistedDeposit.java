package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.WristSubsystem;

@Config
public class DriverAssistedDeposit extends CommandBase {

    public static long depositWaitTime = 1500;

    private ElapsedTime timer;
    private long startMillis;
    private ArmSubsystem arm;
    private WristSubsystem wrist;

    private boolean waitToRetract = true;

    public DriverAssistedDeposit(ArmSubsystem a, WristSubsystem w, ElapsedTime timer) {
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
        wrist.deposit();
    }

    @Override
    public void execute() {
        if (waitToRetract && ((long) timer.milliseconds()) - startMillis >= depositWaitTime) {
            wrist.center();
            arm.center();
            waitToRetract = false;
        }
    }

    @Override
    public boolean isFinished() {return !waitToRetract;}

    @Override
    public void end(boolean interrupted) {
    }
}