package org.firstinspires.ftc.teamcode.backend.bespokeauto;


import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.PIDController;

public class AutoDTController {

    // Putting this here so that I might not forget: center of field is (0, 0), coords are in inches
    // positive y axis faces away from driver, positive x axis faces to the driver's right

    private final DrivetrainSubsystem dt;
    private ElapsedTime timer;
    private double lastTime;
    private Pose2d currPos;
    private Pose2d targetPos;
    private PIDController anglePID = new PIDController();

    public AutoDTController(DrivetrainSubsystem dt, Pose2d startPose, ElapsedTime timer) {
        this.dt = dt;
        this.timer = timer;
        lastTime = timer.milliseconds();
        currPos = startPose;
        targetPos = startPose;
    }

    public void update() {

    }


}
