package org.firstinspires.ftc.teamcode.vuforia;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class SkyStoneLocalizer {

    private VuforiaTrackables Skystone;

    private Position lastPosition = Position.INVALID_POSITION;

    public void init(HardwareMap hardwareMap) {
        WebcamName cam = hardwareMap.get(WebcamName.class, "cam");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "ASHFs7r/////AAABmbSiCe6Dq03flLd+mIyA0Hsp+FgwD3zZqmqX01p8K9eqFha+bq9YuPgTU4hPEPKUwxfYw1u5aYTuEtoXw/2shsdd7e1Cu0rLJ5iHnFlxwYLB1BFjZrIHHTqoNC6oh3nQy4SAwz7q9XULkx5IwZGVcZynRXDmdi+dWLxqovRMmQ5yTbI0Smh1IvMPFsPVGTVRqHZuRPVymZAFuUUD5aIsK0CuYjljG2DDCK+2NWq7flga/KCGx7OdaUUL8Jem9bfHbxmMbOv06+rPPWLCjy74c5CJtqTfL24U9tda2UB/v0KFMlVzKvzXEt0Et0OkIBcgn9S+aaV7xYy7vEJ25ofYwG4hEhmuLMsxwmtKRGDC3eDf";

        parameters.cameraName = cam;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Skystone = vuforia.loadTrackablesFromAsset("Skystone");

     //   Skystone.activate();
    }

    public void activate(){
        Skystone.activate();
    }

    private Position getPosition() {
        VuforiaTrackable stone = Skystone.get(0);
        boolean visible = ((VuforiaTrackableDefaultListener) stone.getListener()).isVisible();
        if (visible) {
            OpenGLMatrix value = ((VuforiaTrackableDefaultListener) stone.getListener()).getUpdatedRobotLocation();
            if (value == null) {
                return Position.INVALID_POSITION;
            }
            double x = value.getTranslation().get(0);
            double y = value.getTranslation().get(1);

            return new Position(x, y);
        } else {
            return Position.INVALID_POSITION;
        }
    }

    public Position getCurrentPosition() {

        Position position = getPosition();
        if (position.isValidPosition()) {
            lastPosition = position;
        }

        return lastPosition;
    }
}
