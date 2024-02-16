package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class IntakeSubsystem extends SubsystemBase {

    public DcMotorImpl motor;

    public ServoImpl rightDropdownServo;

    public static double dropdownUpPos = 1.00;
    public static double dropdownDownPos = 0.70;
    public static double dropdownPosDelta = 0.05;
    private static double dropdownMaxDownPos = dropdownDownPos+4*dropdownPosDelta; // TODO

    public static double power = 0.75;

    private double currentSpeed = 0.0;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        motor = ahwMap.get(DcMotorImpl.class, "IntakeMotor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setPower(currentSpeed);

        rightDropdownServo = ahwMap.get(ServoImpl.class, "RightIntakeDropdownServo");
        rightDropdownServo.setPosition(dropdownUpPos);
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        this.init(aTimer, ahwMap);
    }

    public double getCurrentSpeed() {return currentSpeed;}
    public double getCurrentDropdownPos() {return rightDropdownServo.getPosition();}

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

    public void lowerDropdown() {
        lowerDropdown(0);
    }

    public void lowerDropdown(int pixelsMissed) {
        if (pixelsMissed > 4) {pixelsMissed = 4;} else if (pixelsMissed < 0) {pixelsMissed = 0;}
        rightDropdownServo.setPosition(dropdownDownPos+dropdownPosDelta*pixelsMissed);
    }

    public void raiseDropdown() {
        rightDropdownServo.setPosition(dropdownUpPos);
    }

}
