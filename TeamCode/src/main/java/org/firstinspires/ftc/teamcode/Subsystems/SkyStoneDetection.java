package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Robot.HardwareSkyStone;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class SkyStoneDetection {

    public VuforiaTrackable stoneTarget;
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6.00f * mmPerInch;
    private static final float stoneZ = 2.00f * mmPerInch;



    public SkyStoneDetection(HardwareSkyStone hw){
        VuforiaTrackables targetsSkystone = hw.vuforia.loadTrackablesFromAsset("Skystone");
        stoneTarget = targetsSkystone.get(0);
        stoneTarget.setName("Stone Target");
        stoneTarget.setLocation(OpenGLMatrix.translation(0, 0, stoneZ).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).setPhoneInformation(hw.robotFromCamera, hw.vuforiaParameters.cameraDirection);
        targetsSkystone.activate();
    }
}

