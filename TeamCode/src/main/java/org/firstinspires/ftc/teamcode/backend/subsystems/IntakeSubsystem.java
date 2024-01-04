package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class IntakeSubsystem extends SubsystemBase {

    public DcMotorImpl motor;

    public static double power = 0.75;

    private double currentSpeed = 0.0;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        motor = ahwMap.get(DcMotorImpl.class, "IntakeMotor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setPower(currentSpeed);
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        this.init(aTimer, ahwMap);
    }

    public double getCurrentSpeed() {return currentSpeed;}

    public void setSpeed(double speed) {
        currentSpeed = Math.min(Math.max(speed, -1.0), 1.0);
        motor.setPower(currentSpeed);
    }

    public void intake() {setSpeed(power);}
    public void hold() {setSpeed(0.0);}
    public void outtake() {setSpeed(-power);}

    public void toggleIntake() {
        if (getCurrentSpeed() == power) {setSpeed(0.0);
        } else {setSpeed(power);}
    }
    public void toggleOuttake() {
        if (getCurrentSpeed() == -power) {setSpeed(0.0);
        } else {setSpeed(-power);}
    }

}
