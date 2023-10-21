package org.firstinspires.ftc.teamcode.backend.cv.pipelines;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.backend.cv.PoleDetection;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;

import kotlin.NotImplementedError;

public class PoleLocalizationPipeline extends OpenCvPipeline {

    private final Mat CALIB_PARAMS = Mat.zeros(3, 3, CvType.CV_32FC1);
    private final Mat DIST_COEFFS  = Mat.zeros(1, 5, CvType.CV_32FC1);
    private final Mat IDENTITY     = Mat.eye(3, 3, CvType.CV_32FC1);

    private double fx = 822.317/2; // 670.779069; // 1078.03779?
    private double fy = 822.317/2; // 674.806148; // 1084.50988?
    private final double cx = 319.495/2; // 0.37500; // 361.418117; // 580.850545?
    private double cy = 242.502/2; // 0.64583; // 153.041358; // 245.959325?
    // private final Mat camera_matrix = new Mat(3, 3, );

    private Mat ans = new Mat();
    private Mat crMat = new Mat();
    private Mat cbMat = new Mat();
    private Mat avgMat = new Mat();
    private Mat poleBinaryMat = new Mat();

    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(4, 4));

    public Scalar poleColor = new Scalar(120);
    public Scalar minPoleColor = new Scalar(0);

    private static final int CAMERA_HEIGHT = 86;
    private static final int POLE_WIDTH = 27; // TODO tune this experimentally. desmos link: https://www.desmos.com/calculator/bk4shor7v4

    private static final Scalar CONTOUR_DISPLAY_COLOR = new Scalar(0, 0, 255);
    private static final Scalar BBOX_DISPLAY_COLOR = new Scalar(0, 255, 0);
    private static final Scalar BASE_DISPLAY_COLOR = new Scalar(255, 0, 0);
    private static final int CONTOUR_DISPLAY_THICKNESS = 1;

    private ArrayList<PoleDetection> detections = new ArrayList<PoleDetection>();

    public PoleLocalizationPipeline() {}
    public PoleLocalizationPipeline(Telemetry t) {}

    @Override
    public void init(Mat frame) {
        cy -= (240-frame.size().height)/2;
        CALIB_PARAMS.put(0, 0, fx);
        CALIB_PARAMS.put(1, 1, fy);
        CALIB_PARAMS.put(0, 2, cx);
        CALIB_PARAMS.put(1, 2, cy);
        CALIB_PARAMS.put(2, 2, 1);
        DIST_COEFFS.put(0, 0, -0.0449369);
        DIST_COEFFS.put(0, 1, 1.17277);
        DIST_COEFFS.put(0, 4, -3.6324);
    }

    @Override
    public synchronized Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, ans, Imgproc.COLOR_RGB2YCrCb);
        ArrayList<MatOfPoint> contoursList = findPoleContours(ans);

        ArrayList<Point[]> tops = getPoleTopEdge(contoursList);

        detections.clear();

        for (Point[] top : tops) {
            ArrayList<Point> points = new ArrayList<>(Arrays.asList(top));
            ArrayList<double[]> pointAngles = getPointAngles(points);
            double poleWidthAngle = abs(pointAngles.get(0)[0]-pointAngles.get(2)[0]);
            double poleHeightAngle = abs(pointAngles.get(1)[1]);
            double poleTopDist = POLE_WIDTH/(2*Math.tan(poleWidthAngle/2));
            double poleTrueDist = Math.cos(poleHeightAngle)*poleTopDist;
            detections.add(new PoleDetection((int)poleTrueDist, Math.PI/2-pointAngles.get(1)[0]));
        }


        for (Point[] top : tops) {
            for (Point point : top) {
                Imgproc.drawMarker(ans, point, BASE_DISPLAY_COLOR, 3, CONTOUR_DISPLAY_THICKNESS * 4);
            }
        }

        /* TODO use this code when analyzing the pole's base
        ArrayList<Point> bases = getPoleBases(contoursList);
        ArrayList<double[]> baseAngles = getPointAngles(bases);


        for (double[] base : baseAngles) {
            telemetry.addData("Horiz", base[0]*180/Math.PI);
            telemetry.addData("Vert", base[1]*180/Math.PI);
        }
        Imgproc.drawContours(ans, contoursList, -1, CONTOUR_DISPLAY_COLOR, CONTOUR_DISPLAY_THICKNESS);
        for (Point base : bases) {
            Imgproc.drawMarker(ans, base, BASE_DISPLAY_COLOR, 3, CONTOUR_DISPLAY_THICKNESS*4);
        }
        */
        return ans;
    }

    public synchronized ArrayList<PoleDetection> getDetections() {return (ArrayList<PoleDetection>)detections.clone();}

    public synchronized void debug(Telemetry t) {
        for (PoleDetection d : (ArrayList<PoleDetection>)detections.clone()) {
            if (d == null) {break;}
            t.addLine();
            t.addData("Distance", d.getR());
            t.addData("Heading", d.getTheta()*180/Math.PI);
        }
    }

    private ArrayList<Point[]> getPoleTopEdge(ArrayList<MatOfPoint> poleContours) {
        ArrayList<Point[]> tops = new ArrayList<>();
        for (MatOfPoint contour : poleContours) {
            RotatedRect bbox = getBoundingRect(contour);
            tops.add(getPoleTopEdge(bbox));
        }
        return tops;
    }

    private ArrayList<Point> getPoleBases(ArrayList<MatOfPoint> poleContours) {
        ArrayList<Point> bases = new ArrayList<>();
        for (MatOfPoint contour : poleContours) {
            RotatedRect bbox = getBoundingRect(contour);
            bases.add(getPoleBase(bbox));
        }
        return bases;
    }

    private ArrayList<MatOfPoint> findPoleContours(Mat imageYCrCb) {
        ArrayList<MatOfPoint> ans = new ArrayList<>();
        Core.extractChannel(imageYCrCb, crMat, 1);
        Core.extractChannel(imageYCrCb, cbMat, 2);
        Core.addWeighted(crMat, 0.5, cbMat, 0.5, 0.0, avgMat);
        Core.inRange(avgMat, minPoleColor, poleColor, poleBinaryMat);
        morphMask(poleBinaryMat, poleBinaryMat);
        Imgproc.findContours(poleBinaryMat, ans, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        return ans;
    }

    private ArrayList<double[]> getPointAngles(ArrayList<Point> points) {
        MatOfPoint2f pointsMat = new MatOfPoint2f(points.toArray(new Point[0]));
        Calib3d.undistortPoints(pointsMat, pointsMat, CALIB_PARAMS, DIST_COEFFS, IDENTITY, CALIB_PARAMS);
        ArrayList<double[]> ans = new ArrayList<>();
        for (Point point : pointsMat.toArray()) {
            double horiz = Math.acos(fx/(Math.sqrt(pow(point.x-cx, 2) + pow(fx, 2))))*(point.x-cx)/abs(point.x-cx);
            double vert = Math.acos(fy/(Math.sqrt(pow(point.y-cy, 2) + pow(fy, 2))))*(point.y-cy)/-abs(point.y-cy);
            ans.add(new double[]{horiz, vert});
        }
        return ans;
    }

    private RotatedRect getBoundingRect(MatOfPoint contour) {
        /**
         * This is really just a wrapper that calls Imgproc.minAreaRect on the input, but it does
         * some type-casting too, which is nicer.
         */
        return Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
    }

    private Point getPoleBase(RotatedRect poleBbox) {
        Point[] points = new Point[4];
        poleBbox.points(points);
        Arrays.sort(points, (a, b) -> Double.compare(b.y, a.y));
        Point base = new Point((points[0].x + points[1].x)/2, (points[0].y + points[1].y)/2);
        return base;
    }

    private Point[] getPoleTopEdge(RotatedRect poleBbox) {
        Point[] points = new Point[4];
        poleBbox.points(points);
        Arrays.sort(points, (a, b) -> Double.compare(a.y, b.y));
        Point center = new Point((points[0].x + points[1].x)/2, (points[0].y + points[1].y)/2);
        return new Point[]{points[0], center, points[1]};
    }

    private void morphMask(Mat input, Mat output)
    {
        /**
         * Apply some erosion and dilation for noise reduction
         */

        Imgproc.erode(input, output, erodeElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    private static void drawRotatedRect(RotatedRect rect, Mat drawOn)
    {
        /**
         * Draws a rotated rect by drawing each of the 4 lines individually
         */

        Point[] points = new Point[4];
        rect.points(points);

        for(int i = 0; i < 4; ++i)
        {
            Imgproc.line(drawOn, points[i], points[(i+1)%4], BBOX_DISPLAY_COLOR, CONTOUR_DISPLAY_THICKNESS*2);
        }
    }

}
