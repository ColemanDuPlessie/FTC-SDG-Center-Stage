package org.firstinspires.ftc.teamcode.backend.cv;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

public class PoleDetection {

    public int x;
    public int y;

    public PoleDetection(int x, int y) {
        this.x = x;
        this.y = y;
    }

    public PoleDetection(int r, double theta) {
        /**
         * Constructs a PoleDetection from polar coordinates. The camera is assumed to be (0, 0).
         * Theta is in radians, 0 at the positive X axis, and increases counterclockwise.
         */
        this.x = (int)(r*cos(theta));
        this.y = (int)(r*sin(theta));
    }

    public int getX() {return x;}
    public int getY() {return y;}
    public int getR() {return (int)sqrt(x*x+y*y);}
    public double getTheta() {return atan2(y, x);}

    public PoleDetection plus(int x, int y) {return new PoleDetection(this.x+x, this.y+y);}
    public PoleDetection plus(PoleDetection other) {return new PoleDetection(this.x+other.x, this.y+other.y);}
    public PoleDetection minus(PoleDetection other) {return new PoleDetection(this.x-other.x, this.y-other.y);}
}
