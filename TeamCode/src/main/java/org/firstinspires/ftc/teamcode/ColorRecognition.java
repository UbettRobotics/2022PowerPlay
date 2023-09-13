package org.firstinspires.ftc.teamcode;

import com.vuforia.Rectangle;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Date;
import java.util.List;

public class ColorRecognition extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    Mat mat2 = new Mat();
    Date date = new Date();
    Mat image;
    List<MatOfPoint> contours = new ArrayList<MatOfPoint>();


    final int MAX_CONTOURS = 10;//Must be higher than expected number of contours

    public enum Color {
        Red,
        Blue,
        Green
    }

    private int LOCX = 160;
    private int LOCY = 150;
    private Point p11 = new Point(LOCX,LOCY);
    private Point p12 = new Point(LOCX-10,LOCY-10);

    //This is the enum that is used throughout the pipeline (lowercase variable)
    //It is type 'Side' (as opposed to a long or boolean)
    private Color color;

    //HSV color parameters, determine w/ Python live update program
    private int hueMin = 0;
    private int hueMax = 179;
    private int satMin = 75; //68
    private int satMax = 255;
    private int valMin = 80;
    private int valMax = 239;
    private ArrayList<Double> coneAreaArray;
    private ArrayList<MatOfPoint> conts = new ArrayList<MatOfPoint>();
    static double PERCENT_COLOR_THRESHOLD = 0.4;
    private static int thresh = 25;
    private double cont_count = 0;
    private Rect rect1;
    private double output_conts = 1;
    private double previous_cont = 1;


    public ColorRecognition(Telemetry t) {
        telemetry = t;
    }

    //We use EasyOpenCv, but the docs for what this does will be OpenCv docs

    @Override
    public Mat processFrame(Mat input) {
        //converts frame to HSV colorspace

        conts.clear();
        cont_count = 0;
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //the range of colors to filter.
        Scalar lowHSV = new Scalar(hueMin, satMin, valMin);
        Scalar highHSV = new Scalar(hueMax, satMax, valMax);

        Core.inRange(mat, lowHSV, highHSV, mat);
        Imgproc.findContours(mat, conts, mat2, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


        Mat image_original;
        Point p1,p4;
        p1 = new Point(0, 0);
        p4 = new Point(10, 10);
//        Rect rectCrop = new Rect(p1.x, p1.y , (p4.x-p1.x+1), (p4.y-p1.y+1));
//        Mat image_output= input.submat(rectCrop);
        for(int i = 0; i < conts.toArray().length; i++) {
            double area = Imgproc.contourArea(conts.get(i));
            if(area > thresh) {
                rect1 = Imgproc.boundingRect(conts.get(i));
                if(rect1.y > 190 && rect1.x > 65 && rect1.x < 250) {
                    Imgproc.rectangle(mat, rect1, new Scalar(255,150,0), 5, 8, 0);
                    //Imgproc.rectangle(mat, new  , new Scalar(255, 150, 0), 10, 8, 0);
                    cont_count++;
                    output_conts = cont_count - 2;
                }

            }
        }
        getCount();


        telemetry.addData("#: ", output_conts);
        telemetry.update();


        return mat;
    }

    public int getCount() {
        int ones = 0;
        int twos = 0;
        int threes = 0;


        int return_val = 1;
        if(ones > twos && ones > threes) return_val = 1;
        if(twos >= ones && twos > threes) return_val = 2;
        else return_val = 3; //for equal cases, assume we are missing contours, not seeing extra;


        if(output_conts < 1 || output_conts > 3){
            output_conts = previous_cont;
        } else {
            previous_cont = output_conts;
        }
        return (int)output_conts;
    }

}


//
//    private ArrayList<Double> getContourArea(Mat mat) {
//        Mat hierarchy = new Mat();
//        image = mat.clone();
//        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
//
//        Imgproc.findContours(image, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//
//        ArrayList<Double> arr = new ArrayList<Double>();
//        conarr.clear();
//
//
//        telemetry.update();
//
//        double minArea = 300;
//        for (int i = 0; i < contours.size(); i++) {
//            Mat contour = contours.get(i);
//            double contourArea = Imgproc.contourArea(contour);
//            //Add any contours bigger than error size (ignore tiny bits) to array of all contours
//            if(contourArea > minArea){
//                arr.add(contourArea);
//                conarr.add(contour);
//                Rect bounding = Imgproc.boundingRect(contour);
//                //Draw a rectangle on preview stream
//                Imgproc.rectangle(image, bounding, new Scalar(80,80,80), 4);
//            }
//        }
//
//
//        side = getConeArea();
//
//        return arr;
//    }
//
//    private Side getConeArea() {
//        int biggestContour = 0;
//        if (coneAreaArray == null) {
//            return Side.RIGHT_SIDE;
//        }
//        double biggestContourArea = 0;
//        //Finds biggest contour, can be assumed to be object of concern
//        for (int i = 0; i < coneAreaArray.size(); i++) {
//            if(coneAreaArray.get(i) > biggestContourArea) {
//                biggestContourArea = coneAreaArray.get(i);
//                biggestContour = i;
//            }
//        }
//        //Change these numbers for size determining
//        Side side = Side.RIGHT_SIDE;
//
//        if(conarr.size() <= 0 || conarr.size() <= biggestContour) {
//            return Side.RIGHT_SIDE;
//        } else {
//            Rect rect = Imgproc.boundingRect(conarr.get(biggestContour));
//            //Draw a rectangle on preview stream
//            Imgproc.rectangle(image, rect, new Scalar(128,128,128), 4);
//            if (rect.x < (mat.cols()/6)) {
//                side = Side.LEFT_SIDE;
//            } else if (rect.x < (2 * mat.cols()/3)) {
//                side = Side.MIDDLE_SIDE;
//            } else {
//                side = Side.RIGHT_SIDE;
//            }
//        }
//
//
//        return side;
//    }
//
//}