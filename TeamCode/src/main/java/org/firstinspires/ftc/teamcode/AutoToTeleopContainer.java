package org.firstinspires.ftc.teamcode;

public class AutoToTeleopContainer {
    private static final AutoToTeleopContainer INSTANCE = new AutoToTeleopContainer();
    public static AutoToTeleopContainer getInstance() {return INSTANCE;}

    public Double forwardsAngleDelta;
    private Integer slidesPosition;
    private Integer intakeSlidesPosition;
    private Integer armPosition;

    private AutoToTeleopContainer() {
    }

    public void setAngleDelta(double toSet) {forwardsAngleDelta = toSet;}

    public Double getAngleDelta() {
        if (forwardsAngleDelta == null) {return null;}
        double ans = forwardsAngleDelta;
        forwardsAngleDelta = null;
        return ans;
    }

    public void setSlidesPosition(int toSet) {slidesPosition = toSet;}

    public Integer getSlidesPosition() {
        if (slidesPosition == null) {return null;}
        int ans = slidesPosition;
        slidesPosition = null;
        return ans;
    }

    public void setIntakeSlidesPosition(int toSet) {intakeSlidesPosition = toSet;}

    public Integer getIntakeSlidesPosition() {
        if (intakeSlidesPosition == null) {return null;}
        int ans = intakeSlidesPosition;
        intakeSlidesPosition = null;
        return ans;
    }

    public void setArmPosition(int toSet) {armPosition = toSet;}

    public Integer getArmPosition() {
        if (armPosition == null) {return null;}
        int ans = armPosition;
        armPosition = null;
        return ans;
    }

}
