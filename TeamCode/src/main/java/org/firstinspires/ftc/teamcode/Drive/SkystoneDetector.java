package org.firstinspires.ftc.teamcode.Drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class SkystoneDetector extends VuforiaCommon {

    VuforiaTrackable stoneTarget;

    private static final float mmPerInch = 25.4f;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible;

    public SkystoneDetector(VuforiaLocalizer localizer, VuforiaLocalizer.CameraDirection cameraDirection) {
        super(localizer, cameraDirection);

        stoneTarget = targetsSkystone.get(0);

        ((VuforiaTrackableDefaultListener)stoneTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, cameraDirection);

        targetsSkystone.activate();
    }

    public boolean skystoneVisible(){

        targetVisible = false;
        if (((VuforiaTrackableDefaultListener)stoneTarget.getListener()).isVisible()){
            targetVisible = true;

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)stoneTarget.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null){
                lastLocation = robotLocationTransform;
            }
        }
        return targetVisible;

//        if (targetVisible) {
//            VectorF translation = lastLocation.getTranslation();
//            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//
//            return new Pose2d(translation.get(0)/mmPerInch, translation.get(1)/mmPerInch, Math.toRadians(rotation.thirdAngle));
//        } else {
//            return null;
//        }
    }
}
