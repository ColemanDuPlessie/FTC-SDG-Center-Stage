package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ArmSubsystem extends SubsystemBase {

    public ServoImpl lServo;
    public ServoImpl rServo;

    public static double downPosition = 0.05;
    public static double downWaitingPosition = 0.07;
    public static double waitingPosition = 0.24;
    public static double upPosition = 0.70;

    public static double altServoOffsetPosition = 0.00;

    private double targetPosition = downPosition;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        lServo = ahwMap.get(ServoImpl.class, "LeftArmServo");
        rServo = ahwMap.get(ServoImpl.class, "RightArmServo");
        down();
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        lServo = ahwMap.get(ServoImpl.class, "LeftArmServo");
        rServo = ahwMap.get(ServoImpl.class, "RightArmServo");
        down();
    }

    public double getTargetPosition() {return targetPosition;}

    public double getPosition() {return lServo.getPosition();}

    public void setTargetPosition(double target) {
        targetPosition = target;
        lServo.setPosition(targetPosition);
        rServo.setPosition(1.0-targetPosition+altServoOffsetPosition);
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
