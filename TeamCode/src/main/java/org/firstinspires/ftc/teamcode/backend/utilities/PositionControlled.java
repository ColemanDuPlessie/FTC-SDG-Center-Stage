package org.firstinspires.ftc.teamcode.backend.utilities;

public interface PositionControlled {

    double getPosition();
    double getTargetPosition();
    void setTargetPosition(double targetPosition);
    void incrementTargetPosition(double increment);
}
