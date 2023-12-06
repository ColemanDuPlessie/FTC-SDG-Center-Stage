package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class WristSubsystem extends SubsystemBase {

    public ServoImpl servo;

    public static double downPosition = 0.05; // TODO
    public static double downWaitingPosition = 0.07; // TODO
    public static double waitingPosition = 0.24; // TODO
    public static double upPosition = 0.70; // TODO

    private double targetPosition = downPosition;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        servo = ahwMap.get(ServoImpl.class, "WristServo");
        down();
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        servo = ahwMap.get(ServoImpl.class, "WristServo");
        down();
    }

    public double getTargetPosition() {return targetPosition;}

    public double getPosition() {return servo.getPosition();}

    public void setTargetPosition(double target) {
        targetPosition = target;
        servo.setPosition(targetPosition);
    }

    public void down() {setTargetPosition(downPosition);}
    public void holding() {setTargetPosition(downWaitingPosition);}
    public void center() {setTargetPosition(waitingPosition);}
    public void deposit() {setTargetPosition(upPosition);}

    public void toggle() {
        if (getTargetPosition() == downWaitingPosition) {
            down();
        } else if (getTargetPosition() == downPosition){
            holding();
        } else if (getTargetPosition() == upPosition){
            center();
        } else {
            deposit();
        }
    }

}
