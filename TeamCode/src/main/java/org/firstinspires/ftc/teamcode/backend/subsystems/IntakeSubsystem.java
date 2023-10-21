package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoToTeleopContainer;
import org.firstinspires.ftc.teamcode.backend.utilities.PositionControlled;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.ArmPIDFController;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.PIDController;

@Config
public class IntakeSubsystem extends SubsystemBase {

    public CRServo servo;

    private double currentSpeed = 0.0;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        servo = ahwMap.get(CRServo.class, "IntakeServo");
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        servo.setPower(currentSpeed);
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        this.init(aTimer, ahwMap);
    }

    public double getCurrentSpeed() {return currentSpeed;}

    public void setSpeed(double speed) {
        currentSpeed = Math.min(Math.max(speed, -1.0), 1.0);
        servo.setPower(currentSpeed);
    }

    public void intake() {setSpeed(1.0);}
    public void hold() {setSpeed(0.0);}
    public void outtake() {setSpeed(-1.0);}

    public void toggleIntake() {
        if (getCurrentSpeed() == 1.0) {setSpeed(0.0);
        } else {setSpeed(1.0);}
    }
    public void toggleOuttake() {
        if (getCurrentSpeed() == -1.0) {setSpeed(0.0);
        } else {setSpeed(-1.0);}
    }

}
