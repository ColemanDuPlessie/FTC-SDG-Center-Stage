package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.WristSubsystem;

@Config
public class SlowIntakeFromStack extends CommandBase {

    public static long dropdownTravelTime = 500;
    public static double posDelta = -0.02;

    private ElapsedTime timer;
    private long startMillis;
    private IntakeSubsystem intake;

    private double startPos;

    private boolean readyToEnd = false;

    public SlowIntakeFromStack(IntakeSubsystem i, ElapsedTime timer) {
        intake = i;
        addRequirements(i);
        this.timer = timer;
    }

    @Override
    public void initialize() {
        this.startMillis = (long) timer.milliseconds();
        startPos = intake.getCurrentDropdownPos();
    }

    @Override
    public void execute() {
        if (((long) timer.milliseconds()) - startMillis >= dropdownTravelTime) {
            intake.setRawDropdownPos(startPos+posDelta);
            readyToEnd = true;
        } else {
            intake.setRawDropdownPos(startPos+posDelta*((timer.milliseconds() - ((double)startMillis))/((double)dropdownTravelTime)));
        }
    }

    @Override
    public boolean isFinished() {return readyToEnd;}

    @Override
    public void end(boolean interrupted) {
    }
}