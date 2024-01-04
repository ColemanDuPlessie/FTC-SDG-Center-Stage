package org.firstinspires.ftc.teamcode.backend.cv;

import android.graphics.Bitmap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicReference;

public class AprilTagProcessorWithDash extends AprilTagProcessorImpl implements CameraStreamSource {

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public AprilTagProcessorWithDash(double fx, double fy, double cx, double cy, DistanceUnit outputUnitsLength, AngleUnit outputUnitsAngle, AprilTagLibrary tagLibrary, boolean drawAxes, boolean drawCube, boolean drawOutline, boolean drawTagID, TagFamily tagFamily, int threads) {
        super(fx, fy, cx, cy, outputUnitsLength, outputUnitsAngle, tagLibrary, drawAxes, drawCube, drawOutline, drawTagID, tagFamily, threads);
    }

    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        super.init(width, height, calibration);
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    public Object processFrame(Mat input, long captureTimeNanos) {
        Bitmap b = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, b);
        lastFrame.set(b);
        return super.processFrame(input, captureTimeNanos);
    }

    public static class Builder {
        private double fx;
        private double fy;
        private double cx;
        private double cy;
        private AprilTagProcessor.TagFamily tagFamily;
        private AprilTagLibrary tagLibrary;
        private DistanceUnit outputUnitsLength;
        private AngleUnit outputUnitsAngle;
        private int threads;
        private boolean drawAxes;
        private boolean drawCube;
        private boolean drawOutline;
        private boolean drawTagId;

        public Builder() {
            this.tagFamily = AprilTagProcessor.TagFamily.TAG_36h11;
            this.tagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();
            this.outputUnitsLength = DistanceUnit.INCH;
            this.outputUnitsAngle = AngleUnit.DEGREES;
            this.threads = 3;
            this.drawAxes = false;
            this.drawCube = false;
            this.drawOutline = true;
            this.drawTagId = true;
        }

        public AprilTagProcessorWithDash.Builder setLensIntrinsics(double fx, double fy, double cx, double cy) {
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
            return this;
        }

        public AprilTagProcessorWithDash.Builder setTagFamily(AprilTagProcessor.TagFamily tagFamily) {
            this.tagFamily = tagFamily;
            return this;
        }

        public AprilTagProcessorWithDash.Builder setTagLibrary(AprilTagLibrary tagLibrary) {
            this.tagLibrary = tagLibrary;
            return this;
        }

        public AprilTagProcessorWithDash.Builder setOutputUnits(DistanceUnit distanceUnit, AngleUnit angleUnit) {
            this.outputUnitsLength = distanceUnit;
            this.outputUnitsAngle = angleUnit;
            return this;
        }

        public AprilTagProcessorWithDash.Builder setDrawAxes(boolean drawAxes) {
            this.drawAxes = drawAxes;
            return this;
        }

        public AprilTagProcessorWithDash.Builder setDrawCubeProjection(boolean drawCube) {
            this.drawCube = drawCube;
            return this;
        }

        public AprilTagProcessorWithDash.Builder setDrawTagOutline(boolean drawOutline) {
            this.drawOutline = drawOutline;
            return this;
        }

        public AprilTagProcessorWithDash.Builder setDrawTagID(boolean drawTagId) {
            this.drawTagId = drawTagId;
            return this;
        }

        public AprilTagProcessorWithDash.Builder setNumThreads(int threads) {
            this.threads = threads;
            return this;
        }

        public AprilTagProcessorWithDash build() {
            if (this.tagLibrary == null) {
                throw new RuntimeException("Cannot create AprilTagProcessor without setting tag library!");
            } else if (this.tagFamily == null) {
                throw new RuntimeException("Cannot create AprilTagProcessor without setting tag family!");
            } else {
                return new AprilTagProcessorWithDash(this.fx, this.fy, this.cx, this.cy, this.outputUnitsLength, this.outputUnitsAngle, this.tagLibrary, this.drawAxes, this.drawCube, this.drawOutline, this.drawTagId, this.tagFamily, this.threads);
            }
        }
    }

}
