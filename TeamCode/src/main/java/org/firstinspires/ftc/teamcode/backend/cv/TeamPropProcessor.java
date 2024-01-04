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

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class TeamPropProcessor implements VisionProcessor, CameraStreamSource {

    public enum PROP_POSITION {
        LEFT,
        CENTER,
        RIGHT
    }

    Mat YCrCb = new Mat();

    private final boolean isBlue;

    private PROP_POSITION currentPos = PROP_POSITION.CENTER;
    private double currentConfidence = 1.0;

    public static double minControlExcedance = 20;

    public static int rightX = 295;
    public static int rightY = 168;
    public static int centerX = 130;
    public static int centerY = 165;
    public static int controlX = 210;
    public static int controlY = 215;
    public static int rightW = 25;
    public static int rightH = 32;
    public static int centerW = 30;
    public static int centerH = 20;
    public static int controlW = 32;
    public static int controlH = 25;

    double targetW = 320; // Height calculated based on assumed 3:4 aspect ratio. Other constants scaled as necessary for resolution.

    final Paint noDetectPaint;
    final Paint detectPaint;

    Telemetry t = null;

    public TeamPropProcessor(Telemetry t) {this(false); this.t = t;}

    public TeamPropProcessor(boolean isBlue)
    {
        this.isBlue = isBlue;

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
        rightX *= scale;
        rightY *= scale;
        rightW *= scale;
        rightH *= scale;
        controlX *= scale;
        controlY *= scale;
        controlW *= scale;
        controlH *= scale;
        centerX *= scale;
        centerY *= scale;
        centerW *= scale;
        centerH *= scale;
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        Rect rightCrop = new Rect(rightX, rightY, rightW, rightH);
        Rect centerCrop = new Rect(centerX, centerY, centerW, centerH);
        Rect controlCrop = new Rect(controlX, controlY, controlW, controlH);

        Mat right = new Mat(YCrCb, rightCrop); // TODO this may be inefficient?
        Mat center = new Mat(YCrCb, centerCrop);
        Mat control = new Mat(YCrCb, controlCrop);

        double rightDetectionValue = processArea(right);
        double centerDetectionValue = processArea(center);
        double controlDetectionValue = processArea(control);
        if (t != null) {
            t.addData("right", rightDetectionValue);
            t.addData("center", centerDetectionValue);
            t.addData("control", controlDetectionValue);
        }

        double rightConfidence = rightDetectionValue-controlDetectionValue;
        double centerConfidence = centerDetectionValue-controlDetectionValue;
        if (rightConfidence > minControlExcedance) {
            currentConfidence = rightConfidence-minControlExcedance;
            currentPos = PROP_POSITION.RIGHT;
        } else if (centerConfidence > minControlExcedance) {
            currentConfidence = centerConfidence-minControlExcedance;
            currentPos = PROP_POSITION.CENTER;
        } else {
            currentConfidence = minControlExcedance-Math.max(centerConfidence, rightConfidence);
            currentPos = PROP_POSITION.LEFT;
        }

        Bitmap b = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, b);
        lastFrame.set(b);

        return YCrCb;
    }

    private double processArea(Mat area) {
        Scalar avg = Core.mean(area);
        if (isBlue) {
            return avg.val[2];
        } else {
            return avg.val[1];
        }
    }

    public PROP_POSITION getCurrentPosition() {
        return currentPos;
    }

    public double getCurrentConfidence() {
        return currentConfidence;
    }

    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (userContext != null) {
            Paint lColor = currentPos == PROP_POSITION.LEFT ? detectPaint : noDetectPaint;
            Paint cColor = currentPos == PROP_POSITION.CENTER ? detectPaint : noDetectPaint;
            Paint rColor = currentPos == PROP_POSITION.RIGHT ? detectPaint : noDetectPaint;
            canvas.drawRect((rightX - rightW)*scaleBmpPxToCanvasPx, (rightY - rightH)*scaleBmpPxToCanvasPx, (rightX + rightW)*scaleBmpPxToCanvasPx, (rightY + rightH)*scaleBmpPxToCanvasPx, rColor);
            canvas.drawRect((centerX-centerW)*scaleBmpPxToCanvasPx, (centerY-centerH)*scaleBmpPxToCanvasPx, (centerX+centerW)*scaleBmpPxToCanvasPx, (centerY+centerH)*scaleBmpPxToCanvasPx, cColor);
            canvas.drawRect((controlX - controlW)*scaleBmpPxToCanvasPx, (controlY - controlH)*scaleBmpPxToCanvasPx,(controlX + controlW)*scaleBmpPxToCanvasPx, (controlY + controlH)*scaleBmpPxToCanvasPx,  lColor);
        }
    }

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

}