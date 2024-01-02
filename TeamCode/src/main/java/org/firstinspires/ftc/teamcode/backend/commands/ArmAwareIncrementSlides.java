package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.WristSubsystem;

@Config
public class ArmAwareIncrementSlides extends ArmAwareSetSlides {

    public ArmAwareIncrementSlides(SlidesSubsystem s, ArmSubsystem a, WristSubsystem w, double posIncrement, ElapsedTime timer) {
        super(s, a, w, Math.max(Math.min((s.getTargetPosition() + posIncrement), 1.0), 0.0), timer);
    }

    public ArmAwareIncrementSlides(SlidesSubsystem s, ArmSubsystem a, WristSubsystem w, double posIncrement, ElapsedTime timer, IntakeSubsystem i) {
        super(s, a, w, Math.max(Math.min((s.getTargetPosition() + posIncrement), 1.0), 0.0), timer, i);
    }

}