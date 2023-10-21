package org.firstinspires.ftc.teamcode.backend.utilities.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class GravityPIDFController extends PIDController {

    protected double kGrav;

    public GravityPIDFController(double kP, double kI, double kD, ElapsedTime elapsedTime, double kGrav) {
        super(kP, kI, kD, elapsedTime);
        this.kGrav = kGrav;
    }

    public double update(double currVal, double targetVal) {
        double PIDPower = super.update(currVal, targetVal);
        return  PIDPower + kGrav;
    }

}
