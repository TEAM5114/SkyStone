package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.Robot.HardwareSkyStone;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class SkyStoneDetection {

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true;
    private static final String VUFORIA_KEY =
            "AXV9/0T/////AAABmdy5WsaLkE9sikFO7Jw6Eq12u+U5LoQH27GNe7jhB/Zkx+5rQBJAtxZDAmzKEXFDKnp2i8ypCE7zm8BFkg8ALmGROdE7c5A5mkc3pHm5fD8qkWAeYajXEZLUIXqUpf9aaMFR7vjqQu4QHOlA487t33Qq1GPf2rDSP94MH6AM+14Rwkf8/s2fR+g0ujNXW4lLZtiRIxdLL27b6H/GyJ66XvdijMYF8Rr2NUFo6j8X5Hm4nPV8j68s30m5bY1Ac6DDv4fJ1NPoEyhMKKmPv2YRdABoCGung9pTQXGNWg1uQIl5Ihft9Pmhohbofu3AhlhoOZHgTFX6cLV9EoWT9+BMWj0Cvrks+gpHsNBL0vcsJaGD";

    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6.00f * mmPerInch;
    private static final float stoneZ = 2.00f * mmPerInch;

    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    public SkyStoneDetection(HardwareSkyStone hw){

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(hw.cameraMonitorViewID);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackable stoneTarget = this.vuforia.loadTrackablesFromAsset("Skystone").get(0);
        stoneTarget.setName("Stone Target");
        stoneTarget.setLocation(OpenGLMatrix.translation(0, 0, stoneZ).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        if (PHONE_IS_PORTRAIT){
            phoneXRotate = 90;
        }

        final float CAMERA_FORWARD_DISPLACEMENT = 0.00f * mmPerInch;
        final float CAMERA_VERTICAL_DISPLACEMENT = 0.00f * mmPerInch;
        final float CAMERA_LEFT_DISPLACEMENT = 0.00f * mmPerInch;

        OpenGLMatrix robotFromCamera = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
    }
}

