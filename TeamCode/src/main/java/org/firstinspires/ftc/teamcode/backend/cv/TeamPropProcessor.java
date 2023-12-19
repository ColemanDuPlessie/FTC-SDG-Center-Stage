/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.backend.cv;

import android.graphics.Canvas;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class TeamPropProcessor implements VisionProcessor {

    public enum PROP_POSITION {
        LEFT,
        CENTER,
        RIGHT
    }

    private PROP_POSITION currentPos;
    private double currentConfidence;

    private Mat grey = new Mat();
    private final Object detectionsUpdateSync = new Object();

    Mat cameraMatrix;

    int leftX = 50; // TODO this might be a poor system if I need so many doubles. But do I really need a wrapper class, or is there a better way?
    int leftY = 100;
    int centerX = 100;
    int centerY = 80;
    int rightX = 150;
    int rightY = 100;
    int leftW = 20;
    int leftH = 20;
    int centerW = 20;
    int centerH = 20;
    int rightW = 20;
    int rightH = 20;

    double targetW = 320; // Height calculated based on assumed 3:4 aspect ratio. Other constants scaled as necessary for resolution.

    final Paint noDetectPaint;
    final Paint detectPaint;

    private final Object decimationSync = new Object();

    public TeamPropProcessor()
    {
        noDetectPaint = new Paint();
        noDetectPaint.setARGB(80, 255, 0, 0);
        detectPaint = new Paint();
        detectPaint.setARGB(80, 0, 255, 0);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        assert height/width == 3/4; // TODO this should be more elegant
        double scale = width/targetW;
        leftX *= scale;
        leftY *= scale;
        leftW *= scale;
        leftH *= scale;
        rightX *= scale;
        rightY *= scale;
        rightW *= scale;
        rightH *= scale;
        centerX *= scale;
        centerY *= scale;
        centerW *= scale;
        centerH *= scale;
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos)
    {
        // Convert to greyscale
        Rect leftCrop = new Rect(leftX, leftY, leftW, leftH);
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        // TODO actually process it

        return input;
    }

    public PROP_POSITION getCurrentPosition() {
        return currentPos;
    }

    public double getCurrentConfidence() {
        return currentConfidence;
    }

    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (userContext != null) {
            canvas.drawRect((int)(leftX-leftW)*scaleBmpPxToCanvasPx, (int)(leftY-leftH)*scaleBmpPxToCanvasPx, (int)(leftX+leftW)*scaleBmpPxToCanvasPx, (int)(leftY+leftH)*scaleBmpPxToCanvasPx, noDetectPaint);
            canvas.drawRect((int)(centerX-centerW)*scaleBmpPxToCanvasPx, (int)(centerY-centerH)*scaleBmpPxToCanvasPx, (int)(centerX+centerW)*scaleBmpPxToCanvasPx, (int)(centerY+centerH)*scaleBmpPxToCanvasPx, noDetectPaint);
            canvas.drawRect((int)(rightX-rightW)*scaleBmpPxToCanvasPx, (int)(rightY-rightH)*scaleBmpPxToCanvasPx, (int)(rightX+rightW)*scaleBmpPxToCanvasPx, (int)(rightY+rightH)*scaleBmpPxToCanvasPx, noDetectPaint);
        }
    }

}