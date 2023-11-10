package org.firstinspires.ftc.teamcode.backend;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.Subsystem;
// TODO import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.utilities.GamepadWrapper;

import java.util.List;

public abstract class CommandbasedOpmode extends OpMode {

    /**
     * Cancels all previous commands
     */
    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    /**
     * Runs the {@link CommandScheduler} instance
     */
    public void run() {
        CommandScheduler.getInstance().run();
    }

    /**
     * Schedules {@link com.arcrobotics.ftclib.command.Command} objects to the scheduler
     */
    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    /**
     * Registers {@link com.arcrobotics.ftclib.command.Subsystem} objects to the scheduler
     */
    public void register(Subsystem... subsystems) {
        CommandScheduler.getInstance().registerSubsystem(subsystems);
    }

    public static void disable() {
        Robot.disable();
    }

    public static void enable() {
        Robot.enable();
    }

    protected CommandScheduler scheduler = CommandScheduler.getInstance();
    protected ElapsedTime timer = new ElapsedTime();
    protected Robot19397 robot = new Robot19397(timer);
    private List<LynxModule> allHubs;

    public GamepadWrapper pad1;
    public GamepadWrapper pad2;

    @Override
    public void internalPreInit() {
        super.internalPreInit();
        // TODO PhotonCore.enable();
        pad1 = new GamepadWrapper(gamepad1);
        pad2 = new GamepadWrapper(gamepad2);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void internalPostInitLoop() {
        super.internalPostInitLoop();
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }

    @Override
    public void internalPostLoop() {
        super.internalPostLoop();
        scheduler.run();
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }

    @Override
    public final void stop() {
        end();
        reset();
    }

    public void end() {}

}