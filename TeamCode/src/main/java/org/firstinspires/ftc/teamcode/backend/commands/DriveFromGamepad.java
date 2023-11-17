package org.firstinspires.ftc.teamcode.backend.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.backend.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.backend.utilities.GamepadWrapper;

public class DriveFromGamepad extends CommandBase {

    private final DrivetrainSubsystem dt;
    private final GamepadWrapper gamepad;
    private final boolean isFieldCentric;

    private final double minSpeed = 0.25;

    public DriveFromGamepad(DrivetrainSubsystem dt, GamepadWrapper gamepad, boolean fieldCentric) {
        this.gamepad = gamepad;
        this.dt = dt;
        addRequirements(dt);
        isFieldCentric = fieldCentric;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double forward = -gamepad.getLeftStickY();
        double turn = gamepad.getRightStickX();
        double strafe = gamepad.getLeftStickX();
        double speed = 1.0 - gamepad.getLeftTrigger() * (1.0-minSpeed);
        if (isFieldCentric) {
            dt.driveFieldCentric(forward, turn, strafe, speed);
        } else {
            dt.driveSimple(forward, turn, strafe, speed);
        }
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean isInterrupted) {}
}
