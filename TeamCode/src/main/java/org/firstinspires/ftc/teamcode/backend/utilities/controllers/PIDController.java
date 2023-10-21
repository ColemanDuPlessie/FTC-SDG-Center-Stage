package org.firstinspires.ftc.teamcode.backend.utilities.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    protected double kP = 8;
    protected double kI = 0;
    protected double kD = 1;

    private double lastError = 0.0;
    private double lastTime;
    private ElapsedTime elapsedTime;
    private double integralSum = 0.0;

    // TODO see if this breaks everything private double integralLimit = 125.0;

    public PIDController(ElapsedTime elapsedTime) {
        this.elapsedTime = elapsedTime;
    }

    public PIDController(double kP, double kI, double kD, ElapsedTime elapsedTime) {
        this.elapsedTime = elapsedTime;
        lastTime = elapsedTime.seconds();
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double update(double currVal, double targetVal) {
        double error = targetVal - currVal;
        double currTime = elapsedTime.seconds();
        double timePassed = currTime - lastTime;
        integralSum += error * timePassed;
        // TODO see above integralSum = Math.max(-integralLimit, Math.min(integralSum, integralLimit));
        double power = kP * error + kI * integralSum + kD * (lastError - error) * timePassed;
        lastError = error;
        lastTime = currTime;
        return Math.min(1.0, Math.max(-1.0, power));
    }

}
