package org.firstinspires.ftc.team6220_2018;

import android.graphics.Bitmap;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class OpenCVGold extends OpenCVPipeline
{
    // The minimum (or maximum) yellow value that is necessary for OpenCV to detect a gold mineral.
    int thresholdVal = 107;

    private boolean showContours = true;
    // To keep it such that we don't have to instantiate a new Mat every call to processFrame,
    // we declare the Mats up here and reuse them. This is easier on the garbage collector.
    private Mat hsv = new Mat();
    private Mat thresholded = new Mat();
    private Mat blurred = new Mat();
    public Mat detectedEdges = new Mat();

    private List<Mat> channels = new ArrayList<>();
    private Mat maskYellow = new Mat();  // Yellow Mask returned by color filter
    private Mat hierarchy  = new Mat();  // hierarchy used by contours
    private Mat displayMat = new Mat(); // Display debug info to the screen (this is what is returned)

    private Rect goldRect = new Rect(0,0,0,0);

    // This is just here so we can expose it later through getContours.
    private List<MatOfPoint> contours = new ArrayList<>();

    public synchronized void setShowCountours(boolean enabled) {
        showContours = enabled;
    }
    public synchronized List<MatOfPoint> getContours() {
        return contours;
    }

    public Rect getGoldRect()
    {
        return goldRect;
    }

    // This is called every camera frame.
    @Override
    public Mat processFrame(Mat rgba, Mat gray)
    {

        rgba.copyTo(displayMat);

        int maxY = rgba.rows();
        int maxX = rgba.cols();

        int integer = maxX + maxY;

        // Filter yellow
        Imgproc.cvtColor(rgba, rgba, Imgproc.COLOR_RGB2YUV);
        Imgproc.GaussianBlur(rgba, rgba, new Size(3, 3), 0);
        channels = new ArrayList<>();
        Core.split(rgba, channels);
        if (channels.size() > 0)
        {
            Imgproc.threshold(channels.get(1), maskYellow, thresholdVal, 255, Imgproc.THRESH_BINARY_INV);
        }

        // Find contours of the yellow mask and draw them to the display mat for viewing

        List<MatOfPoint> contoursYellow = new ArrayList<>();
        Imgproc.findContours(maskYellow, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat, contoursYellow, -1, new Scalar(230, 70, 70), 2);

        double area;
        double maxArea = 0.0;
        Rect rect;
        Rect maxRect = new Rect(0, 0, 0, 0);   // Rect with max area
        Rect temp = null;


        // Loop through the contours and find the contour with max area
        for (MatOfPoint cont : contoursYellow)
        {
            // Get bounding rect of contour
            rect = Imgproc.boundingRect(cont);
            area = Imgproc.contourArea(cont);

            // Make sure rectangles that we identify are outside the crater
            if (area > maxArea && rect.x > Constants.DISCRIMINATION_HEIGHT)
            {
                maxArea = area;
                maxRect = rect;
            }
        }
        goldRect = maxRect;

        Imgproc.rectangle(displayMat, maxRect.tl(), maxRect.br(), new Scalar(0, 0, 255), 2); // Draw rect
        // Draw Current X
        Imgproc.putText(displayMat, "Gold", maxRect.tl(), 0, 1, new Scalar(255, 255, 255));

        // Debug: display bitmap
        Bitmap bmp = null;
        int rows = displayMat.rows();
        int cols = displayMat.cols();
// create a new 4 channel Mat because bitmap is ARGB
        Mat tmp = new Mat(displayMat.rows(), displayMat.cols(), CvType.CV_8U, new Scalar(4));
// convert ROI image from single channel to 4 channel
//        Imgproc.cvtColor(displayMat, tmp, Imgproc.COLOR_GRAY2RGBA, 4);
// Initialize bitmap
        bmp = Bitmap.createBitmap(cols, rows, Bitmap.Config.ARGB_8888);
// convert Mat to bitmap
        Utils.matToBitmap(tmp, bmp);
        // First, we change the colorspace from RGBA to HSV, which is usually better for color
        //Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);
        // Then, we threshold our hsv image so that we get a black/white binary image where white
        // is the blues listed in the specified range of values
        // you can use a program like WPILib GRIP to find these values, or just play around.
        //Core.inRange(hsv, new Scalar(90, 128, 30), new Scalar(170, 255, 255), thresholded);

        /*
        // we blur the thresholded image to remove noise
        // there are other types of blur like box blur or gaussian which can be explored.
        //Imgproc.blur(thresholded, thresholded, new Size(3, 3));
        Imgproc.GaussianBlur(gray, gray, new Size(7,7), 2, 2);

        // Contrast Limited Adaptive Histogram Equalization
        // CLAHE clahe = Imgproc.createCLAHE(2);


        Imgproc.adaptiveThreshold(gray, gray, 200, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY_INV, 9, 2);
        //Imgproc.dilate(gray, gray, Imgproc.getStructuringElement(Imgproc.CV_SHAPE_CROSS, new Size(5,5) ) );

        // canny detector, with ratio of lower:upper threshold of 3:1
        //Imgproc.Canny(gray, gray, 80, 160);

        if (showContours) {
            Mat circles = new Mat();

            Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT,
                    1,  // asp ratio
                    gray.rows()/8, // change this value to detect circles with different distances to each other
                    140.0,  // upper threshold for Canny edge detection
                    60.0,  // bigger, fewer circles
                    10, // min radius
                    200); // max radius

            for (int x = 0; x < circles.cols(); x++) {
                double[] c = circles.get(0, x);
                Point center = new Point(Math.round(c[0]), Math.round(c[1]));
                // circle center
                Imgproc.circle(rgba, center, 1, new Scalar(0, 100, 100), 4, 8, 0);
                // circle outline
                int radius = (int) Math.round(c[2]);
                Imgproc.circle(rgba, center, radius, new Scalar(10, 10, 255), 4, 8, 0);
            }
        }
        else
        {
            return gray;
        }
        // canny detector, with ratio of lower:upper threshold of 3:1
        //Imgproc.Canny(thresholded, rgba, 80, 160);


        return rgba; // display the image seen by the camera
        //return thresholded; // display the image seen by the camera
        */
        return displayMat;
    }

}
