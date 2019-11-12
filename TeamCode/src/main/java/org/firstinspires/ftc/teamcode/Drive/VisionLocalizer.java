package org.firstinspires.ftc.teamcode.Drive;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Config
public class VisionLocalizer extends VuforiaCommon implements Localizer {

    public static double LOW_FREQ_WEIGHT = 0.50;

    private static final float mmPerInch = 25.4f;

    private List<VuforiaTrackable> allTrackables;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    private Localizer highFrequencyLocalizer;
    private Pose2d poseEstimate;

    public VisionLocalizer(VuforiaLocalizer vuforiaLocalizer, VuforiaLocalizer.CameraDirection cameraDirection, Localizer highFrequencyLocalizer){
        super(vuforiaLocalizer, cameraDirection);
        this.highFrequencyLocalizer = highFrequencyLocalizer;

        allTrackables = new ArrayList<>();
        for (int i = 1; i < 13; i++){
            allTrackables.add(targetsSkystone.get(i));
        }

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, cameraDirection);
        }

        targetsSkystone.activate();
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate(){
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d){
        this.poseEstimate = pose2d;
        highFrequencyLocalizer.setPoseEstimate(pose2d);
    }

    @Override
    public void update(){
//        highFrequencyLocalizer.setPoseEstimate(poseEstimate);
        highFrequencyLocalizer.update();
        Pose2d highFrequencyPoseEstimate = highFrequencyLocalizer.getPoseEstimate();

        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if(((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()){
                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null){
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        if (targetVisible) {
            VectorF translation = lastLocation.getTranslation();
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

            Pose2d lowFreqPoseEstimate = new Pose2d(translation.get(0)/mmPerInch, translation.get(1)/mmPerInch, Math.toRadians(rotation.thirdAngle));
            poseEstimate = lowFreqPoseEstimate.times(LOW_FREQ_WEIGHT).plus(highFrequencyPoseEstimate.times(1 - LOW_FREQ_WEIGHT));
            highFrequencyLocalizer.setPoseEstimate(poseEstimate);
        } else {
            poseEstimate = highFrequencyPoseEstimate;
        }
    }
}
