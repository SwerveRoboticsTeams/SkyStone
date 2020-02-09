package org.firstinspires.ftc.team417_2019.ImageRecognition;


import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


public class OpenCVDetect extends OpenCVPipeline
{
    // Matrices needed for processing
    private Mat hsv = new Mat();

    private Mat threshold = new Mat();

    private Mat rgb = new Mat();

    private Mat nonYellowMask = new Mat();

    private Mat nonBlackMask = new Mat();

    private Mat cropped = new Mat();

    private Mat hierarchy  = new Mat();

    boolean showContours = true;

    private Mat displayMat = new Mat();

    // other variables for detection
    double maxArea;

    private Rect blackRect = new Rect(0,0,0,0);

    public double thresh = 33;

    public double p1 = blackRect.x;

    public double p2 = blackRect.x + blackRect.width;

    public double width = displayMat.size().width;

    // special mask
    public Mat YellowDetection( Mat src , Mat nonYellowMask) {

        Scalar lower = new Scalar(15,150,150);
        Scalar upper = new Scalar(40,255,255);

        Imgproc.cvtColor(src, hsv, Imgproc.COLOR_BGR2HSV);

        Core.inRange(hsv,lower,upper,nonYellowMask);

        Imgproc.GaussianBlur(hsv,hsv,new Size(3,3),0);

        Size kernelSize = new Size(6,6);
        Point anchor = new Point(-1,-1);
        int iterations = 2;

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize);

        //Imgproc.erode(nonYellowMask, nonYellowMask, kernel, anchor, iterations);

        Imgproc.dilate(nonYellowMask, nonYellowMask, kernel, anchor,iterations);

        return nonYellowMask;
    }

    public Mat BlackDetection( Mat src, Mat nonBlackMask){

        // darker conditions
        Scalar lower = new Scalar(0,100,100);
        Scalar upper = new Scalar(thresh,255,255);
        // lighter conditions
        //Scalar lower = new Scalar(0,0,0);
        //Scalar upper = new Scalar(79,145,166);

        Imgproc.GaussianBlur(cropped,cropped,new Size(3,3),0);

        Core.inRange(cropped,lower,upper,nonBlackMask);

        Size kernelSize = new Size(6,6);
        Point anchor = new Point(-1,-1);
        int iterations = 2;

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize);

        //Imgproc.erode(nonYellowMask, nonYellowMask, kernel, anchor, iterations);

        Imgproc.dilate(nonBlackMask, nonBlackMask, kernel, anchor,iterations);




















        return nonBlackMask;
    }

    // skystone coordinates
    List<MatOfPoint> contours;

    // This is called every camera frame
    @Override
    public Mat processFrame(Mat rgba, Mat gray)
    {
        // copy frame to a separate Mat
        rgba.copyTo(displayMat);
        // convert rgba to bgr so that it can be processed
        Imgproc.cvtColor(rgba, rgb, Imgproc.COLOR_RGBA2BGR);
        // find the yellow
        Mat process = YellowDetection(rgb, nonYellowMask);
        // create an empty list of yellow contours
        contours = new ArrayList<>();
        // save the contours
        Imgproc.findContours(process, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // find the largest contour
        maxArea = 0.0;
        Rect rect;
        Rect maxRect = new Rect(0,0,0,0);

        for (MatOfPoint contour: contours){
            rect = Imgproc.boundingRect(contour);
            double area = Imgproc.contourArea(contour);
            if (area > maxArea){
                maxArea = area;
                maxRect = rect;
            }
        }

        // get (x,y) of the yellow rectangle
        int y = maxRect.y;
        // find size of displayMat for extending the rectangle
        width = displayMat.size().width;
        //width = maxRect.width;
        double height = maxRect.height;
        // crop the camera's view such that we only see what we want

        //displayMat = displayMat.submat()
        cropped = displayMat.submat(y, (int) (y + height), 0, (int)(0 + width));

        cropped.copyTo(displayMat);

        // convert to something that can be worked with
        Imgproc.cvtColor(cropped, cropped,  Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_BGR2YUV);

        threshold = BlackDetection(cropped, nonBlackMask);

        List<MatOfPoint> contoursBlack = new ArrayList<>();

        Imgproc.findContours(threshold, contoursBlack, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // find the largest contour
        maxArea = 0.0;

        for (MatOfPoint contour: contoursBlack){
            rect = Imgproc.boundingRect(contour);
            double area = Imgproc.contourArea(contour);
            if (area > maxArea){
                maxArea = area;
                blackRect = rect;
            }
        }

        double blackY = blackRect.y;
        double blackBeight = blackRect.height;

        Rect lastStep = new Rect(blackRect.x, blackRect.y,144,blackRect.height);

        if (showContours){
            Imgproc.rectangle(displayMat, lastStep.tl(), lastStep.br(), new Scalar(0, 255, 0), 5);
            //Imgproc.rectangle(displayMat, maxRect.tl(), maxRect.br(), new Scalar(0,255,0), 5);
            Imgproc.putText(displayMat, "Width"+ lastStep.width, lastStep.tl(),0,1,new Scalar(255,255,255));
            //Imgproc.putText(displayMat, "Hei" + blackRect.height, blackRect.br(),0,1,new Scalar(255,255,255));
            p1 = lastStep.x;
            p2 = lastStep.x + lastStep.width;
        }



        return displayMat;
    }

    // setter methods
    public void setShowContours(boolean set) {
        showContours = set;
    }
    // getter methods
    public synchronized List<MatOfPoint> getContours() {
        return contours;
    }
    public Rect getSksytonePosition() {
        return blackRect;
    }


}
