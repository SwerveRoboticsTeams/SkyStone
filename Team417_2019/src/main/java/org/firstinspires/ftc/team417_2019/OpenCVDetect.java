package org.firstinspires.ftc.team417_2019;


import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
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

    private Mat rgb = new Mat();

    private Mat nonYellowMask = new Mat();

    private boolean showContours = true;

    private Mat hierarchy  = new Mat();

    private Mat displayMat = new Mat();

    // special mask
    public Mat YellowDetection( Mat src , Mat nonYellowMask) {

        Scalar lower = new Scalar(15,150,150);
        Scalar upper = new Scalar(50,255,255);

        Imgproc.cvtColor(src, hsv, Imgproc.COLOR_BGR2HSV);

        Core.inRange(hsv,lower,upper,nonYellowMask);

        Imgproc.GaussianBlur(hsv,hsv,new Size(3,3),0);

        Size kernelSize = new Size(6,6);
        Point anchor = new Point(-1,-1);
        int iterations = 2;

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, kernelSize);

        //Imgproc.erode(nonYellowMask, nonYellowMask, kernel, anchor, iterations);

        Imgproc.dilate(nonYellowMask, nonYellowMask, kernel, anchor,iterations);

        return nonYellowMask;
    }


    // skystone coordinates
    private Rect skystone = new Rect(0,0,0,0);

    List<MatOfPoint> contoursYellow;

    // This is called every camera frame
    @Override
    public Mat processFrame(Mat rgba, Mat gray)
    {
        rgba.copyTo(displayMat);

        Imgproc.cvtColor(rgba, rgb, Imgproc.COLOR_RGBA2BGR);

        Mat process = YellowDetection(rgb, nonYellowMask);

        contoursYellow = new ArrayList<>();

        Imgproc.findContours(process, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        ArrayList<Rect> rectangles = new ArrayList<Rect>();

        //MatOfPoint2f contoursSpecial = new MatOfPoint2f(contoursYellow);

        /*RotatedRect rect ;
        double maxArea = 0.0;
        RotatedRect yellowRect = new RotatedRect();*/

        Rect rect = new Rect(0,0,0,0);

        for (MatOfPoint contour: contoursYellow){

            double area = Imgproc.contourArea(contour);

            if (area > 1000){
                rect = Imgproc.boundingRect(contour);
                //rectangles.add(rect);
                if(showContours) {
                    Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0, 0, 255), 4);
                }
            }

        }

        double division = (rect.x + rect.width) / 3;
        double marker1 = rect.x + division;
        double marker2 = marker1 + division;
        /*
        // 1st division
        Imgproc.line(displayMat, new Point(marker1, rect.y + rect.height), new Point(marker1, rect.y), new Scalar(0,255,0), 3);
        // 2nd division
        Imgproc.line(displayMat, new Point(marker2, rect.y + rect.height), new Point(marker2, rect.y), new Scalar(0,255,0), 3);
       */
        // 1st rectangle
        // 2nd rectangle
        // 3rd rectangle
        ;

        Rect rect1 = new Rect(rect.x, rect.y, (int) division, rect.height);
        Rect rect2 = new Rect((int)marker1, rect.y, (int) division, rect.height);
        Rect rect3 = new Rect((int)marker2, rect.y, (int) division, rect.height);


        double mean1 = Core.mean(new Mat(displayMat,rect1)).val[0];
        double mean2 = Core.mean(new Mat(displayMat,rect2)).val[0];
        double mean3 = Core.mean(new Mat(displayMat,rect3)).val[0];

        if ( mean1 < mean2 && mean1 < mean3) {
            Imgproc.rectangle(displayMat, new Point(marker1, rect.y + rect.height),new Point(rect.x, rect.y), new Scalar(0,0,0),3);
            skystone = rect1;
        }
        else if ( mean2 < mean1 && mean2 < mean3) {
            Imgproc.rectangle(displayMat, new Point(marker1 + 5, rect.y + rect.height), new Point(marker2, rect.y), new Scalar(0,0,0), 3);
            skystone = rect2;
        }
        else if ( mean3 < mean2 && mean3 < mean1) {
            Imgproc.rectangle(displayMat, new Point(marker2 + 5, rect.y + rect.height), new Point(rect.x + rect.width, rect.y), new Scalar(0,0,0), 3);
            skystone = rect3;
        }
        /*for (Rect rect: rectangles){
            if(showContours) {
                Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0, 0, 255), 4);
            }
        }*/


        //Imgproc.drawContours(displayMat,rectangles,-1,new Scalar(230,70,70),2);
        //int height = yellowRect.y + yellowRect.height;
        //Imgproc.rectangle(displayMat, yellowRect.points(yellowRect.center[]), new Scalar(0,0,255),4); // Draw rect
        //Imgproc.line(displayMat,new Point(yellowRect.y, yellowRect.x), new Point(yellowRect.y, yellowRect.x + yellowRect.width), new Scalar(0,0,255));
       // Imgproc.putText(displayMat, "Height", new Point (yellowRect.y,yellowRect.x),0,2,new Scalar(255,255,255));


        /*
        // create a rectangle to use for the rotation resistant detection
        RotatedRect rect ;
        // create an array of vertices
        Point vertices[] = new Point[8];


        // Loop through the contours that we know are yellow
        for(MatOfPoint cont : contoursYellow){
            // find area
            area = Imgproc.contourArea(cont);
            // convert matofPoint to matofPoint2f
            MatOfPoint2f contour2f = new MatOfPoint2f(cont);
            // find the min area of the rectangle to make it restistant to rotation
            rect = Imgproc.minAreaRect(contour2f);
            // find vertices
            rect.points(vertices);

            if (area > 800){
                // show them if we want by drawing them on displayMat
                if (showContours) {
                    for (int j = 0; j < 4; j++){
                        Imgproc.line(displayMat, vertices[j], vertices[(j+1)%4], new Scalar(230,70,70));
                    }
                }
            }
        }

        // display skystone
        // Imgproc.rectangle(displayMat, maxRect.tl(), maxRect.br(), new Scalar(0,0,255),2); // Draw rect

        // Draw text on screen
        // Imgproc.putText(displayMat, "Gold", maxRect.tl(),0,1,new Scalar(255,255,255));

        // create a new 4 channel Mat because bitmap is ARGB
        // Mat tmp = new Mat (displayMat.rows(), displayMat.cols(), CvType.CV_8U, new Scalar(4));
*/
        return displayMat;
    }

    // getter methods
    public void setShowCountours(boolean enabled) {
        showContours = enabled;
    }
    public synchronized List<MatOfPoint> getContours() {
        return contoursYellow;
    }
    public Rect getSksytonePosition() {
        return skystone;
    }


}
