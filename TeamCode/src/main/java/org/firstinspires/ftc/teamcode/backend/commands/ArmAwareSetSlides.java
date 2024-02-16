package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.WristSubsystem;

@Config
public class ArmAwareSetSlides extends CommandBase {

    public static double changeoverPosition = 0.25;
    public static long minArmTravelWaitTime = -1000;
    public static long maxArmTravelWaitTime = 750;
    private long armTravelWaitTime;

    private ElapsedTime timer;
    private double targetPos;
    private long startMillis;
    private SlidesSubsystem slides;
    private ArmSubsystem arm;
    private WristSubsystem wrist;

    private IntakeSubsystem intake = null;
    public static long intakeRuntime = 500;
    public static double intakePower = -0.30;
    public static double intakeEnablePoint = 0.10;
    private long intakeEnableTime;
    private boolean waitToIntake = false;
    private boolean waitToDisableIntake = false;

    private boolean waitToLower = false;
    private boolean waitToRaise = false;

    public ArmAwareSetSlides(SlidesSubsystem s, ArmSubsystem a, WristSubsystem w, double targetPos, ElapsedTime timer) {
        slides = s;
        arm = a;
        wrist = w;
        addRequirements(s);
        addRequirements(a);
        addRequirements(w);
        this.targetPos = targetPos;
        this.timer = timer;
    }

    public ArmAwareSetSlides(SlidesSubsystem s, ArmSubsystem a, WristSubsystem w, double targetPos, ElapsedTime timer, IntakeSubsystem i) {
        this(s, a ,w, targetPos, timer);
        if (targetPos < intakeEnablePoint) {
            intake = i;
            addRequirements(i);
        }
    }

    @Override
    public void initialize() {
        this.startMillis = (long) timer.milliseconds();
        armTravelWaitTime = (long)(minArmTravelWaitTime + (maxArmTravelWaitTime-minArmTravelWaitTime)*(1-(slides.getPosition()-changeoverPosition)/(1-changeoverPosition)));
        double startPos = slides.getPosition();
        if (startPos < changeoverPosition && targetPos > changeoverPosition) {
            arm.holding();
            wrist.traveling();
            waitToRaise = true;
            slides.setTargetPosition(targetPos);
        } else if (startPos > changeoverPosition && targetPos < changeoverPosition) {
            arm.holding();
            wrist.holding();
            waitToLower = true;
        } else if (targetPos < changeoverPosition) {
            arm.holding();
            wrist.holding();
            slides.setTargetPosition(targetPos);
        } else {
            arm.center();
            wrist.center();
            slides.setTargetPosition(targetPos);
        }

        if (targetPos < intakeEnablePoint && intake != null) {
            waitToIntake = true;
        }
        if (startPos < intakeEnablePoint && targetPos >= intakeEnablePoint && intake != null) {
            intake.raiseDropdown();
        }
    }

    @Override
    public void execute() {
        if (waitToIntake && slides.getPosition() < intakeEnablePoint) {
            intake.setSpeed(intakePower);
            intakeEnableTime = (long) timer.milliseconds();
            waitToIntake = false;
            waitToDisableIntake = true;
        }
        if (waitToDisableIntake && ((long) timer.milliseconds()) - intakeEnableTime >= intakeRuntime) {
            intake.hold();
            waitToDisableIntake = false;
        }
        if (waitToLower && ((long) timer.milliseconds()) - startMillis >= armTravelWaitTime) {
            slides.setTargetPosition(targetPos);
            waitToLower = false;
        }
        if (waitToRaise && slides.getPosition() >= changeoverPosition) {
            arm.center();
            wrist.center();
            waitToRaise = false;
        }
    }

    @Override
    public boolean isFinished() {return !(waitToLower || waitToRaise || waitToIntake || waitToDisableIntake);}

    @Override
    public void end(boolean interrupted) {
    }

    public void debug(Telemetry t) {
        t.addData("Target position:", targetPos);
    }
}