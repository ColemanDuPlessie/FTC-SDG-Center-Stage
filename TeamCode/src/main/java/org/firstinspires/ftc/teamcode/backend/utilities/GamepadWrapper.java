package org.firstinspires.ftc.teamcode.backend.utilities;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadWrapper {
    /*
    This class is primarily used so that I can pass higher-order functions to commands. It is
    necessary because the Gamepad class has
     */

    protected final Gamepad g;

    public GamepadWrapper(Gamepad gamepad) {
        g = gamepad;
    }

    public boolean getA() { return g.a; }
    public boolean getB() { return g.b; }
    public boolean getX() { return g.x; }
    public boolean getY() { return g.y; }

    public boolean getDpadUp() { return g.dpad_up; }
    public boolean getDpadRight() { return g.dpad_right; }
    public boolean getDpadDown() { return g.dpad_down; }
    public boolean getDpadLeft() { return g.dpad_left; }

    public boolean getLeftBumper() { return g.left_bumper;}
    public boolean getRightBumper() { return g.right_bumper;}
    public double getLeftTrigger() { return g.left_trigger;}
    public double getRightTrigger() { return g.right_trigger;}

    public boolean getLeftStickButton() { return g.left_stick_button;}
    public double getLeftStickX() { return g.left_stick_x;}
    public double getLeftStickY() { return g.left_stick_y;}
    public boolean getRightStickButton() { return g.right_stick_button;}
    public double getRightStickX() { return g.right_stick_x;}
    public double getRightStickY() { return g.right_stick_y;}
}
