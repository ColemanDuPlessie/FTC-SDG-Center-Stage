package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ArmSubsystem extends SubsystemBase {

    public ServoImpl servo;

    public static double downPosition = 0.42;
    public static double waitingPosition = 0.60;
    public static double upPosition = 1.00;


    private double targetPosition = downPosition;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        servo = ahwMap.get(ServoImpl.class, "IntakeServo");
        down();
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        servo = ahwMap.get(ServoImpl.class, "ArmServo");
        down();
    }

    public double getTargetPosition() {return targetPosition;}

    public double getPosition() {return servo.getPosition();}

    public void setTargetPosition(double target) {
        targetPosition = target;
        servo.setPosition(targetPosition);
    }

    public void down() {setTargetPosition(downPosition);}
    public void center() {setTargetPosition(waitingPosition);}
    public void deposit() {setTargetPosition(upPosition);}

    public void toggle() {
        if (getTargetPosition() == waitingPosition) {
            down();
        } else {
            center();
        }
    }

}
