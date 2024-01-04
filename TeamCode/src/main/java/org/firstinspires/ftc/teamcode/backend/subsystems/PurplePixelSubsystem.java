package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class PurplePixelSubsystem extends SubsystemBase {

    public ServoImpl servo;

    public static double waitingPosition = 0.50;
    public static double activePosition = 0.18;

    private double targetPosition = waitingPosition;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        servo = ahwMap.get(ServoImpl.class, "PurplePixelServo");
        hold();
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        servo = ahwMap.get(ServoImpl.class, "PurplePixelServo");
        hold();
    }

    public double getTargetPosition() {return targetPosition;}

    public double getPosition() {return servo.getPosition();}

    public void setTargetPosition(double target) {
        targetPosition = target;
        servo.setPosition(targetPosition);
    }

    public void hold() {setTargetPosition(waitingPosition);}
    public void activate() {setTargetPosition(activePosition);}

}
