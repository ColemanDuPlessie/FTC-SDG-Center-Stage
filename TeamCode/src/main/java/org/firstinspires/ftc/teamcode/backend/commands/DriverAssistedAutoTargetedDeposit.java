package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SetDrivingStyle;
import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.WristSubsystem;

@Config
public class DriverAssistedAutoTargetedDeposit extends CommandBase {

    public static long armTravelWaitTime = 500;
    public static long depositWaitTime = 1500; // Counting from 0, not from the previous event

    private ElapsedTime timer;
    private long startMillis;
    private ArmSubsystem arm;
    private WristSubsystem wrist;

    private SlidesSubsystem s;
    private IntakeSubsystem i;

    private boolean waitToDeposit = true;
    private boolean waitToRetract = true;

    public DriverAssistedAutoTargetedDeposit(ArmSubsystem a, WristSubsystem w, ElapsedTime timer) {
        arm = a;
        wrist = w;
        addRequirements(a);
        addRequirements(w);
        this.timer = timer;
    }

    public DriverAssistedAutoTargetedDeposit(ArmSubsystem a, WristSubsystem w, ElapsedTime timer, SlidesSubsystem s, IntakeSubsystem i) {
        this(a, w, timer);
        this.s = s;
        this.i = i;
    }

    @Override
    public void initialize() {
        /**
         * Assumes that the arm and wrist are already in the pre-deposit position (deposit for
         * the arm and center for the wrist).
         */
        this.startMillis = (long) timer.milliseconds();
        arm.depositAutoTargeted();
        wrist.ready();
    }

    @Override
    public void execute() {
        if (waitToDeposit && ((long) timer.milliseconds()) - startMillis >= armTravelWaitTime) {
            wrist.deposit();
            waitToDeposit = false;
        }
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
        if (SetDrivingStyle.depositAutoRetract && s != null && i != null) {
            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new WaitCommand(500), new ArmAwareSetSlides(s, arm, wrist, 0.0, timer, i)));
        }
    }
}