package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoToTeleopContainer;
import org.firstinspires.ftc.teamcode.backend.utilities.PositionControlled;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.PIDController;

@Config
public class SlidesSubsystem extends SubsystemBase implements PositionControlled {

    public DcMotor motor;

    private PIDController PIDF;

    public static int minPosition = -50; // Added to ensure complete retraction
    public static int maxPosition = 3700; // Can theoretically go up to 3825. Lowered for safety

    public static int armRaisePosition = 600;
    public static int armLowerPosition = 1200;

    public static double kP = 0.007; // TODO tune this
    public static double kI = 0.0000;
    public static double kD = 0.000065;
    public static double kG = 0.25;
    public static double maxPower = 0.75;

    private int targetPosition;

    private int startPosition;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        motor = ahwMap.get(DcMotor.class, "SlidesMotor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startPosition = motor.getCurrentPosition();
        targetPosition = 0;
        PIDF = new PIDController(kP, kI, kD, aTimer);
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        motor = ahwMap.get(DcMotor.class, "SlidesMotor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (isTeleop) {
            Integer position = AutoToTeleopContainer.getInstance().getSlidesPosition();
            if (position == null) {
                startPosition = motor.getCurrentPosition();
            } else { startPosition = position;}
        } else {
            startPosition = motor.getCurrentPosition();
            AutoToTeleopContainer.getInstance().setSlidesPosition(startPosition);
        }
        targetPosition = 0;
        PIDF = new PIDController(kP, kI, kD, aTimer);
    }

    public double getPosition() {return ((double)(motor.getCurrentPosition()-startPosition)-minPosition)/(double)(maxPosition-minPosition);}

    public double getTargetPosition() {return ((double)(targetPosition-startPosition)-minPosition)/(double)(maxPosition-minPosition);}

    public void setTargetPosition(double target) {
        targetPosition = (int)(target * (maxPosition-minPosition) + minPosition);
        targetPosition = Math.min(Math.max(targetPosition, minPosition), maxPosition);
    }

    public void incrementTargetPosition(double increment) {
        targetPosition += (int)(increment * (maxPosition-minPosition));
        targetPosition = Math.min(Math.max(targetPosition, minPosition), maxPosition);
    }

    @Override
    public void periodic() {
        int currentPosition = motor.getCurrentPosition();
        double actualPower = Math.min(maxPower, Math.max(PIDF.update(motor.getCurrentPosition()-startPosition, targetPosition) * maxPower, -maxPower)) + kG;
        motor.setPower(actualPower);
    }

}
