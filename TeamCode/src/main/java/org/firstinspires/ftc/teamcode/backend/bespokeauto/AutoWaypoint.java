package org.firstinspires.ftc.teamcode.backend.bespokeauto;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

public class AutoWaypoint {
    public Pose2d pose;
    public Vector2d velo;

    public AutoWaypoint(Pose2d pose) {
        this.pose = pose;
        this.velo = new Vector2d(0, 0);
    }

    public AutoWaypoint(Pose2d pose, Vector2d velo) {
        this.pose = pose;
        this.velo = velo;
    }

}
