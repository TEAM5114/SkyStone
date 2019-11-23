package org.firstinspires.ftc.teamcode.Drive;

import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Drive.DriveConstants.encoderTicksToInches;

public class MecanumREV extends MecanumBase {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;

    private static final String VUFORIA_KEY =
            "AXV9/0T/////AAABmdy5WsaLkE9sikFO7Jw6Eq12u+U5LoQH27GNe7jhB/Zkx+5rQBJAtxZDAmzKEXFDKnp2i8ypCE7zm8BFkg8ALmGROdE7c5A5mkc3pHm5fD8qkWAeYajXEZLUIXqUpf9aaMFR7vjqQu4QHOlA487t33Qq1GPf2rDSP94MH6AM+14Rwkf8/s2fR+g0ujNXW4lLZtiRIxdLL27b6H/GyJ66XvdijMYF8Rr2NUFo6j8X5Hm4nPV8j68s30m5bY1Ac6DDv4fJ1NPoEyhMKKmPv2YRdABoCGung9pTQXGNWg1uQIl5Ihft9Pmhohbofu3AhlhoOZHgTFX6cLV9EoWT9+BMWj0Cvrks+gpHsNBL0vcsJaGD";
//    private LocalizerVuforia vuforia;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE =
        VuforiaLocalizer.CameraDirection.BACK;
//    private static final boolean useWebcam = true;
//    WebcamName webcamName = null;


    public MecanumREV(HardwareMap hardwareMap){
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        //If hub is mounted vertically, remap the IMU axes so that the z axis
        //points upward.  ex: BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        /**
         * If not using built-in velocity PID, comment out the following block
         * and tune kStatic and kA
         */
        for (DcMotorEx motor : motors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection((DcMotor.Direction.REVERSE));

        setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(28, 8, 0));

        //ex: setLocalizer(new LocalizerVuforia(...));
//        int cameraMonitorViewID = hardwareMap.appContext.getResources()
//                .getIdentifier("cameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
//        LocalizerVuforia.Parameters vuforiaParameters = new LocalizerVuforia.Parameters(cameraMonitorViewID);
//        if (useWebcam) {
//            webcamName = hardwareMap.get(WebcamName.class, "webcam");
//            vuforiaParameters.cameraName = webcamName;
//        }
//        vuforiaParameters.vuforiaLicenseKey = VUFORIA_KEY;
//        vuforiaParameters.cameraDirection = LocalizerVuforia.CameraDirection.BACK;
//        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);
        VuforiaLocalizer vuforia = createVuforia(hardwareMap);
        setLocalizer(new LocalizerVuforia(vuforia, CAMERA_CHOICE,
                new MecanumLocalizer(this, true)));

        TFObjectDetector tfod = createTfod(hardwareMap, vuforia);

        detector = new SkystoneDetector(tfod);
    }

    private VuforiaLocalizer createVuforia(HardwareMap hardwareMap){

        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewID", "id", hardwareMap.appContext.getPackageName()
        );

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(
                cameraMonitorViewID);

        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        return ClassFactory.getInstance().createVuforia(parameters);
    }

    private TFObjectDetector createTfod(HardwareMap hardwareMap, VuforiaLocalizer vuforia){
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        TFObjectDetector tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset("Skystone.tflite", "Stone", "Skystone");
        return tfod;
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients){
        for (DcMotorEx motor : motors){
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions(){
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities(){
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3){
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading(){
        return imu.getAngularOrientation().firstAngle;
    }
}
