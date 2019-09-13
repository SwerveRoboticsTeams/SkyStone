package org.firstinspires.ftc.team417_2018;

import android.graphics.Bitmap;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.CLAHE;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class OpenCVDetect extends OpenCVPipeline
{
    // based on true or false show the contours around the yellow cube and circles
    private boolean showContours = true;
    // To keep it such that we don't have to instantiate a new Mat every call to processFrame,
    // we declare the Mats up here and reuse them. This is easier on the garbage collector.
    private Mat hsv = new Mat();
    private Mat thresholded = new Mat();
    private Mat blurred = new Mat();
    public Mat detectedEdges = new Mat();

    // threshold value for sensing yellow
    private int threshold = 70;

    private List<Mat> channels = new ArrayList<>();
    private Mat maskYellow = new Mat();  // Yellow Mask returned by color filter
    private Mat hierarchy  = new Mat();  // hierarchy used by contours
    private Mat displayMat = new Mat(); // Display debug info to the screen (this is what is returned)

    private Rect goldRect = new Rect(0,0,0,0);

    // this is just here so we can expose it later threw getContours.
    private List<MatOfPoint> contours = new ArrayList<>();

    double area;
    double maxArea = 0.0;

    // we want to show the contours on the RC phone
    public synchronized void setShowCountours(boolean enabled) {
        showContours = enabled;
    }
    public synchronized List<MatOfPoint> getContours() {
        return contours;
    }

    public Rect getGoldRect() {
        return goldRect;
    }

    public void setThreshold(int threshold){
        this.threshold = threshold;
    }

    // This is called every camera frame
    @Override
    public Mat processFrame(Mat rgba, Mat gray)
    {
        // copy the Mat you are working with to display mat so it will show it on the phone
        rgba.copyTo(displayMat);
        // we use YUV instead of RGB to recognize color (Y= luminance, u/v = chrominance)
        Imgproc.cvtColor(rgba, rgba, Imgproc.COLOR_RGB2YUV);
        // blur the image so the color is easier to detect
        Imgproc.GaussianBlur(rgba,rgba,new Size(3,3),0);
        // sum the pixels in the channels
        channels = new ArrayList<>();
        Core.split(rgba, channels);
        // if the channels contain something
        if(channels.size() > 0)
        {
            // find yellow
            Imgproc.threshold(channels.get(1), maskYellow, threshold, 255, Imgproc.THRESH_BINARY_INV);
            //Imgproc.adaptiveThreshold(channels.get(1), maskYellow, 255.0, threshold, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, 5, 2);
        }

        // Debug: display bitmap
        Bitmap bmp = null;
        int rows = displayMat.rows();
        int cols = displayMat.cols();


        // draw contours around the yellow points on the screen
        List<MatOfPoint> contoursYellow = new ArrayList<>();
        Imgproc.findContours(maskYellow, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        // show them if we want by drawing them on displayMat
        if (showContours)
            Imgproc.drawContours(displayMat,contoursYellow,-1,new Scalar(230,70,70),2);

        // create a new Rectangle to prepare to find largest contour which will be the yellow cube
        maxArea = 0.0;
        Rect rect ;
        Rect maxRect = new Rect(0,0,0,0);
        Rect temp = null;

        // Loop through the yellow contours and find the contour with max area
        for(MatOfPoint cont : contoursYellow){

            // Get bounding rect of contour
            rect = Imgproc.boundingRect(cont);
            area = Imgproc.contourArea(cont);
            if (area > maxArea){
                maxArea = area;
                maxRect = rect;
            }
        }
        goldRect = maxRect;

        // display rectangle
        Imgproc.rectangle(displayMat, maxRect.tl(), maxRect.br(), new Scalar(0,0,255),2); // Draw rect

        // Draw text on screen
        Imgproc.putText(displayMat, "Gold", maxRect.tl(),0,1,new Scalar(255,255,255));

        // create a new 4 channel Mat because bitmap is ARGB
        Mat tmp = new Mat (displayMat.rows(), displayMat.cols(), CvType.CV_8U, new Scalar(4));

        return displayMat;
    }


}
