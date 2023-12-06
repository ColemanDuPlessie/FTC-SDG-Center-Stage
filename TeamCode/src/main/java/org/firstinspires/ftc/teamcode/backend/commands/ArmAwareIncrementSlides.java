package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;

@Config
public class ArmAwareIncrementSlides extends ArmAwareSetSlides {

    public ArmAwareIncrementSlides(SlidesSubsystem s, ArmSubsystem a, double posIncrement, ElapsedTime timer) {
        super(s, a, Math.max(Math.min((s.getTargetPosition() + posIncrement), 1.0), -1.0), timer);
    }

}