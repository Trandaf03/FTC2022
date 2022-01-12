package org.firstinspires.ftc.teamcode.utilities.rec;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class recunoastere extends OpenCvPipeline {

    Telemetry telemetry;
    Mat mat = new Mat();
    public recunoastere(Telemetry t){
        telemetry = t;
    }
    public enum duckPosition{
        LEFT,
        RIGHT,
        CENTER,
        NOT_FOUND
    }
    private duckPosition location;
    //TODO verification triangles,set correct camera pos
    static final Rect LEFT_VERIF = new Rect(
            new Point(60,50), new Point(120,50)
    );
    static final Rect CENTER_VERIF = new Rect(
            new Point(140,50), new Point(200,50)
    );
    static final Rect RIGHT_VERIF = new Rect(
            new Point(220,50), new Point(280,50)
    );

    static final double PERCENT_COLOR_THRESHOLD = 0.4; // TODO tune this to find the duck
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2HSV);

        //TODO this makes the img gray,modify it if u want
        Scalar lowHSV = new Scalar(23,50,70);
        Scalar highHSV = new Scalar(32,255,255);

        Core.inRange(mat,lowHSV,highHSV,mat);
        Mat left = mat.submat(LEFT_VERIF);
        Mat center = mat.submat(CENTER_VERIF);
        Mat right = mat.submat(RIGHT_VERIF);

        double leftValue = Core.sumElems(left).val[0] / LEFT_VERIF.area() / 255;
        double centerValue = Core.sumElems(center).val[0] / CENTER_VERIF.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_VERIF.area() / 255;

        left.release();
        center.release();
        right.release();

        telemetry.addData("Left raw value",(int)Core.sumElems(left).val[0]);
        telemetry.addData("Center raw value",(int)Core.sumElems(center).val[0]);
        telemetry.addData("Right raw value",(int)Core.sumElems(right).val[0]);


        boolean duckLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean duckCenter = centerValue > PERCENT_COLOR_THRESHOLD;
        boolean duckRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if(duckLeft && duckCenter && duckRight){
            location = recunoastere.duckPosition.NOT_FOUND;
            //telemetry.addLine("n-am gasit rata");
        }
        if(duckLeft) {
            location = duckPosition.LEFT;
            //telemetry.addLine("rata e in stanga");
        } else if(duckCenter){
            location = duckPosition.CENTER;
            //telemetry.addLine("rata e in centru");
        } else {
            location = duckPosition.RIGHT;
            //telemetry.addLine("rata e in dreapta");
        }
        telemetry.update();

        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);

        Scalar colorNothing = new Scalar(255,0,0);
        Scalar colorRata = new Scalar(0,255,0);

        Imgproc.rectangle(mat,LEFT_VERIF,location == duckPosition.LEFT ? colorRata:colorNothing);
        Imgproc.rectangle(mat,CENTER_VERIF,location == duckPosition.CENTER ? colorRata:colorNothing);
        Imgproc.rectangle(mat,RIGHT_VERIF,location == duckPosition.RIGHT ? colorRata:colorNothing);

        return mat;
    }

    public duckPosition getLocation(){
        return location;
    }
}
