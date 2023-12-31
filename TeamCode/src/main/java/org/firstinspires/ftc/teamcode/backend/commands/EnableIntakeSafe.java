package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.WristSubsystem;

@Config
public class EnableIntakeSafe extends CommandBase {

    public static long armTravelWaitTime = 150;

    private ElapsedTime timer;
    private long startMillis;
    private IntakeSubsystem intake;
    private ArmSubsystem arm;
    private WristSubsystem wrist;

    private boolean waitToEnable = true;
    private boolean isReversed = false;

    public EnableIntakeSafe(IntakeSubsystem i, ArmSubsystem a, WristSubsystem w, ElapsedTime timer) {
        intake = i;
        arm = a;
        wrist = w;
        addRequirements(i);
        addRequirements(a);
        addRequirements(w);
        this.timer = timer;
    }

    public EnableIntakeSafe(IntakeSubsystem i, ArmSubsystem a, WristSubsystem w, ElapsedTime timer, boolean isReversed) {
        intake = i;
        arm = a;
        wrist = w;
        addRequirements(i);
        addRequirements(a);
        addRequirements(w);
        this.isReversed = isReversed;
        this.timer = timer;
    }

    @Override
    public void initialize() {
        this.startMillis = (long) timer.milliseconds();
        arm.down();
        wrist.down();
    }

    @Override
    public void execute() {
        if (waitToEnable && ((long) timer.milliseconds()) - startMillis >= armTravelWaitTime) {
            if (isReversed) {
                intake.outtake();
            } else {
                intake.intake();
            }
            waitToEnable = false;
        }
    }

    @Override
    public boolean isFinished() {return !(waitToEnable);}

    @Override
    public void end(boolean interrupted) {
    }
}