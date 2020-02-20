package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "OpenCV", group = "grup")
public class Test extends LinearOpMode {

    OpenCvCamera cam1;

    public static String pos = "Initialising";

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware robot = new Hardware();
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam1 = OpenCvCameraFactory.getInstance().createWebcam(robot.web, cameraMonitorViewId);

        for(int i=1;i<100;i++)
         cam1.openCameraDevice();

        telemetry.addLine("Camera is ready");
        telemetry.update();
        waitForStart();

        cam1.setPipeline(new proces());
        cam1.startStreaming(640,480);

        while(opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("The stone is in the", pos);
            telemetry.update();
            if(gamepad1.a)
                cam1.stopStreaming();
        }
    }
}

class proces extends OpenCvPipeline
{
    double middle=0;
    double right=0;
    double left=0;

  @Override
    public Mat processFrame(Mat input)
    {
        int offsetY=60;
        int offsetX=-180;
        int x=30;
        int y=20;
        // Imgproc.medianBlur(input,input,35);
        Scalar colors,colorm,colord;
        colors=colord=colorm=  new Scalar(62,238,69);
        Imgproc.threshold(input, input, 102, 255, Imgproc.THRESH_BINARY_INV);
        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2GRAY);
        middle = Core.sumElems(input.submat(input.height()/2-2*x+offsetY,input.height()/2+2*x+offsetY,input.width()/2-x+offsetX,input.width()/2+x+offsetX)).val[0];
        right =  Core.sumElems(input.submat(input.height()/2-2*x-4*x-y+offsetY,input.height()/2+2*x-4*x-y+offsetY,input.width()/2-x+offsetX,input.width()/2+x+offsetX)).val[0];
        left =  Core.sumElems(input.submat(input.height()/2-2*x+4*x+y+offsetY,input.height()/2+2*x+4*x+y+offsetY,input.width()/2-x+offsetX,input.width()/2+x+offsetX)).val[0];
//        Imgproc.putText(
//                input,
//                String.valueOf(middle),
//                new Point(input.width()/2-x,input.height()/2+2*x),
//                Imgproc.FONT_HERSHEY_PLAIN,
//                3,
//                new Scalar(0,0,0),
//                3
//
//        );
//        Imgproc.putText(
//                input,
//                String.valueOf(left),
//                new Point(input.width()/2-x,input.height()/2+2*x+4*x+y),
//                Imgproc.FONT_HERSHEY_PLAIN,
//                3,
//                new Scalar(0,0,0),
//                3
//
//        );
//        Imgproc.putText(
//                input,
//                String.valueOf(right),
//                new Point(input.width()/2-x,input.height()/2+2*x-4*x-y),
//                Imgproc.FONT_HERSHEY_PLAIN,
//                3,
//                new Scalar(0,0,0),
//                3
//
//        );
        if(middle>=left && middle >=right)
        {
            colorm= new Scalar(255,0,0);
            Test.pos="middle";

        }
        else{
            if(left>=middle && left>=right)
            {
                colors=new Scalar(255,0,0);
                Test.pos="left";
            }
            else
            {
                colord=new Scalar(255,0,0);
                Test.pos= "right";
            }

        }
        Imgproc.rectangle(
                input,
                new Point(input.width()/2-x+offsetX,input.height()/2+2*x+offsetY),
                new Point(input.width()/2+x+offsetX,input.height()/2-2*x+offsetY),
                colorm,
                5
        );

        Imgproc.rectangle(
                input,
                new Point(input.width()/2-x+offsetX,input.height()/2.0+2*x+4*x+y+offsetY),
                new Point(input.width()/2+x+offsetX,input.height()/2.0-2*x+4*x+y+offsetY),
                colors,
                5
        );

        Imgproc.rectangle(
                input,
                new Point(input.width()/2-x+offsetX,input.height()/2.0+2*x-4*x-y+offsetY),
                new Point(input.width()/2+x+offsetX,input.height()/2.0-2*x-4*x-y+offsetY),
                colord,
                5
        );

        return input;
    }
}