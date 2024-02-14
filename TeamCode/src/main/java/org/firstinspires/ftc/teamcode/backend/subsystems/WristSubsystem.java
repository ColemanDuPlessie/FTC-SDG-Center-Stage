package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class WristSubsystem extends SubsystemBase {

    public ServoImpl servo;

    public static double downPosition = 0.49;
    public static double downWaitingPosition = 0.42;
    public static double slidesTravelPosition = 0.30;
    public static double waitingPosition = 0.70;
    public static double readyPosition = 0.90;
    public static double upPosition = 0.70;

    private double targetPosition = downPosition;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        servo = ahwMap.get(ServoImpl.class, "WristServo");
        holding();
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        servo = ahwMap.get(ServoImpl.class, "WristServo");
        holding();
    }

    public double getTargetPosition() {return targetPosition;}

    public double getPosition() {return servo.getPosition();}

    public void setTargetPosition(double target) {
        targetPosition = target;
        servo.setPosition(targetPosition);
    }

    public void down() {setTargetPosition(downPosition);}
    public void holding() {setTargetPosition(downWaitingPosition);}
    public void traveling() {setTargetPosition(slidesTravelPosition);}
    public void center() {setTargetPosition(waitingPosition);}
    public void ready() {setTargetPosition(readyPosition);}
    public void deposit() {setTargetPosition(upPosition);}

    public void toggle() {
        if (getTargetPosition() == downWaitingPosition || getTargetPosition() == slidesTravelPosition) {
            down();
        } else if (getTargetPosition() == downPosition) {
            holding();
        } else if (getTargetPosition() == upPosition) {
            ready();
        } else if (getTargetPosition() == readyPosition) {
            deposit();
        } else if (getTargetPosition() == waitingPosition) {
            ready();
        } else {
            center();
        }
    }

}
