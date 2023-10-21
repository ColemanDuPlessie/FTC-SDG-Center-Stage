package org.firstinspires.ftc.teamcode.backend.utilities.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmPIDFController extends PIDController {

    protected double kGrav;
    protected double horizTicks;
    protected double vertTicks;

    public ArmPIDFController(double kP, double kI, double kD, ElapsedTime elapsedTime, double kGrav, double horizTicks, double vertTicks) {
        super(kP, kI, kD, elapsedTime);
        this.kGrav = kGrav;
        this.horizTicks = horizTicks;
        this.vertTicks = vertTicks;
    }

    public double update(double currVal, double targetVal) {
        double PIDPower = super.update(currVal, targetVal);
        double gravityFF = kGrav * Math.cos((currVal - horizTicks)/(vertTicks-horizTicks)*Math.PI/2);
        return  PIDPower + gravityFF;
    }

}
