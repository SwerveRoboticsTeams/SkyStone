package org.firstinspires.ftc.team417_2019;

import android.graphics.Bitmap;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class OpenCVDetect2018Test extends OpenCVPipeline
{
    // based on true or false show the contours around the yellow cube and circles
    private boolean showContours = true;

    private Mat hLow = new Mat();
    private Mat hHigh = new Mat();

    private Mat rgb = new Mat();
    private Mat hsv = new Mat();
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
        Imgproc.cvtColor(rgba, rgb, Imgproc.COLOR_RGBA2RGB);
        // copy the Mat you are working with to display mat so it will show it on the phone
        rgb.copyTo(displayMat);
        // we use YUV instead of RGB to recognize color (Y= luminance, u/v = chrominance)
        Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);
        // blur image
        Imgproc.GaussianBlur(hsv,hsv, new Size(3,3),0);
        //
        channels = new ArrayList<>();
        Core.split(hsv, channels);
        if (channels.size() > 0)
        {
            Imgproc.threshold(channels.get(0), hLow , 15, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(channels.get(1), maskYellow, 100, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(channels.get(2), maskYellow, 100,255, Imgproc.THRESH_BINARY);
        }
        // sum the pixels in the channels
            //Imgproc.threshold(channels.get(0), maskYellow, threshold, 255, Imgproc.THRESH_BINARY_INV);
            //Imgproc.adaptiveThreshold(channels.get(1), maskYellow, 255.0, threshold, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, 5, 2);


        /*
        // set up a kernel size to take out of frame
        Mat kernel = Mat.ones(5,5, CvType.CV_8U);
        // take out the noise by removing small "kernels"
        Imgproc.erode(rgba, rgba, kernel);
       */

        //Debug: display bitmap
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

        double division = (getGoldRect().x + getGoldRect().width) / 3;
        double marker1 = getGoldRect().x + division;
        double marker2 = marker1 + division;
        // 1st division
        Imgproc.line(displayMat, new Point(marker1, getGoldRect().y), new Point(marker1, getGoldRect().y + getGoldRect().height), new Scalar(0,255,0), 3);
        // 2nd division
        Imgproc.line(displayMat, new Point(marker2, getGoldRect().y), new Point(marker2, getGoldRect().y + getGoldRect().height), new Scalar(0,255,0), 3);

        return displayMat;
    }

}
