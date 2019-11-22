package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "Auto_RED", group = "Team7643")
//@Disabled

public class SkyStone_Auto_RED extends LinearOpMode {

    //**********************VUFORIA STUFF*************************************

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private WebcamName webcamName = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;
    private static final String VUFORIA_KEY = "ARxlubj/////AAABmaAWvvNcaEfatpedWzcPYoNNzXvozddqiaVF0eqgKR71EFvFsUChMN1cTJhTfx7pLvbxNnz338HqcYkMq3vkWg+AWEGUPcomz44FQ6ax2INPKro2ZXiUasw3qjx6yDi7TyD0b5zW8MVBBDJrqlEmCi13JLVWZTtmrG4xAh1LryGISe2tT9fqHV04K2nuynDDRGcT9O3WT4yghlNQ/vWnR3oJkgidaGUCsCV6UqfrLAa94r9Lkd/bSyvr29q8qkS5HXJSjFmlXMabP3mTVNhX2e1mfwm8AGjMRBzJPtyqBp7OJUqADQgP5mDH+Qs4gPeKrtcM1FfNurJyxe4MTaumoJYMoRa6PoY9jlkAlITh5kio";
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private static final float stoneZ = 2.00f * mmPerInch;
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;


    //****************DRIVE AND COMPONENTS**************************************************
    private ElapsedTime tim1 = new ElapsedTime();
    private double timref = tim1.seconds();
    private double timestart = timref;
    private DcMotor LF;
    private DcMotor RF;
    private DcMotor LR;
    private DcMotor RR;
    private DcMotor Lint;
    private DcMotor Rint;
    //private ModernRoboticsI2cColorSensor color = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color");
    private Rev2mDistanceSensor las1;
    private Rev2mDistanceSensor las2;
    private Servo ServoKeyStoneBlue;
    private Servo ServoKeyStoneRed;

    private double avgdist = 0;
    private double avgcnt = 0;
    private double ticsperinch = 480 / 4 / 3.1415926;
    private boolean sawanimage = false;
    private double underthebridge = 48;
    private double returnto2ndstone = 72;
    private int lfencint = 0;
    private int rfencint = 0;
    private int rrencint = 0;
    private int lrencint = 0;
    private int lfenccur = 0;
    private int rfenccur = 0;
    private int rrenccur = 0;
    private int lrenccur = 0;

    ////////****************timer****************************

    ///distances------
    private double BlockAdist=55;
    private double BlockBdist=47;
    private double BlockCdist=39;
    private double RtnDistExtraA=BlockAdist+24;
    private double RtnDistExtraB=BlockBdist+24;
    private double RtnDistExtraC=BlockAdist+20;
    private double StrafeExtraA=0;
    private double StrafeExtraB=0;
    private double StrafeExtraC=4;

    private boolean BlockA=false;
    private boolean BlockB=false;
    private boolean BlockC=false;
    private double MoveToSeeTarget=-10;
    private double MovetoContactState=-15;
    private double MovewhileCaputing=-4;


    ///////****************imu stuff******************************
    private BNO055IMU imu;
    private double angle1 = 0;
    private double angle2 = 0;
    private double err = 0;
    private double kp = 1 / 10;  //calibrate for forward angle correction
    private double ki = 0.1;  //calibrate for forward angle correction
    private double ticsperdegree = 8.6444444444;
    private boolean angleok = false;


    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuparam = new BNO055IMU.Parameters();

        //initializing the imu

        //imuparam = imu.getParameters();
        imuparam.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuparam.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuparam.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuparam.loggingEnabled = true;
        imuparam.loggingTag = "IMU";
        imuparam.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(imuparam);

        //initializing other stuff

        LF = hardwareMap.get(DcMotor.class, "lf");
        RF = hardwareMap.get(DcMotor.class, "rf");
        LR = hardwareMap.get(DcMotor.class, "lr");
        RR = hardwareMap.get(DcMotor.class, "rr");
        Lint = hardwareMap.get(DcMotor.class, "lint");
        Rint = hardwareMap.get(DcMotor.class, "rint");
        //private ModernRoboticsI2cColorSensor color = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color");
        las1 = hardwareMap.get(Rev2mDistanceSensor.class, "las1");
        las2 = hardwareMap.get(Rev2mDistanceSensor.class, "las2");
        ServoKeyStoneBlue = hardwareMap.get(Servo.class, "lks");
        ServoKeyStoneRed = hardwareMap.get(Servo.class, "rks");

        ServoKeyStoneRed.setDirection(Servo.Direction.REVERSE);
        ServoKeyStoneBlue.setDirection(Servo.Direction.FORWARD);

        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        LR.setDirection(DcMotorSimple.Direction.FORWARD);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initializing vuforia

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        //***************************edit the following**************

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 7.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        targetsSkyStone.activate();
        stopandreset();
        Power(0, 0, 0, 0);


        waitForStart();

        ServoKeyStoneRed.setPosition(0.6);
        ServoKeyStoneRed.setPosition(0.5);


        //****move forward enough to see the images***************** (STEP2)
        //stopandreset();  //redundant
        runforward(-10);
        runtopos();
        Power(0.4, 0.4, 0.4, 0.4);
        while (LF.isBusy() || RF.isBusy() || RR.isBusy() || LR.isBusy()) {
        }
        Power(0, 0, 0, 0);


        //**** determine if you can see and image on stone 3 or stone 2 ********* (STEP3)

        //findimage(allTrackables);  //redundant
        timref = tim1.seconds();
        while (!targetVisible && (tim1.seconds() - timref) < 3.0) {
            findimage(allTrackables);
        }


        ///*******END OF VUFORIA - STRAFE TO CORRECT STONE *********************if(sawanimage) {
        if (sawanimage) {
            telemetry.addData("avgdist ", avgdist);
            telemetry.update();
            underthebridge -= (3.5 + avgdist);
            returnto2ndstone -= (avgdist + 3.5);
            if (avgdist < 0) {
                BlockA = true;
                returnto2ndstone -= 3;
            }

            strafe(-3.5 - avgdist);
        } else {
            strafe(-16);
            underthebridge -= 16;
            returnto2ndstone -= 16;
        }

        //reset for next image grab
        targetVisible = false;
        sawanimage = false;

        waitforrobot(0.4);


        ///***approach block gently


        runforward(-15);
        waitforrobot(0.25);

        sleep(500);

        //*****approach block even more gently

        runforward(-4);
        waitforrobot(0.20);
        ServoKeyStoneRed.setPosition(0.89);

        sleep(250);  /// might be needed to secure arm

        telemetry.addData("serv pos ", ServoKeyStoneRed.getController().getServoPosition(1));
        telemetry.addData("2nd angle ", imu.getAngularOrientation().firstAngle * 57.3);
        telemetry.addData("avgdist ", avgdist);
        telemetry.update();


        strafe(-1.5);  //make sure the arm drops into slot
        waitforrobot(0.6); // shouldn't need this

        //*******pull back before turning might need to reduce this
        runforward(10);
        waitforrobot(0.4);


        //**********turn 90 degree to the right
        turneasy(-90);
        waitforrobot(0.35);
        // checkandcorrect(-90);
        // if(!angleok){ waitforrobot(0.35);}

        //***********run under the bridge and drop brick
        motorencint();
        runforward(-underthebridge);
        waitforrobot(0.6);
        ServoKeyStoneRed.setPosition(0.5);
        sleep(250);




        //**********run back to the 2nd stone roughly
        //motorencint();
        runforward(returnto2ndstone);
        waitforrobot(0.6);

        turneasy(90);
        waitforrobot(0.35);

        runforward(4.0);
        waitforrobot(0.6);
        checkandcorrect(0);
        if (!angleok) {
            waitforrobot(1.0);
        }
        if(BlockA) {
            strafe(3);
            waitforrobot(0.4);
        }

        //*************check the position of the keystone and strafe as needed
        //  if(!angleok){ waitforrobot(0.4);}

        timref = tim1.seconds();
        while (!targetVisible && (tim1.seconds() - timref) < 2.0) {
            findimage(allTrackables);
        }
        if (sawanimage) {
            telemetry.addData("avgdist ", avgdist);
            telemetry.update();
            returnto2ndstone -= (avgdist + 3.5);
            strafe(-3.5 - avgdist);
            waitforrobot(0.4);
        }

        //***********************approach and pickup keystone

        runforward(-16);
        waitforrobot(0.4);
        runforward(-4);
        ServoKeyStoneRed.setPosition(0.89);
        waitforrobot(0.2);
        sleep(250);

        strafe(-1.5);  //make sure the arm drops into slot
        waitforrobot(0.6); // shouldn't need this


        //strafe(-1.5);  //make sure the arm drops into slot
        //waitforrobot(0.2); //shouldn't need this

        //**************************/pull back 10 inches may need to reduce
        runforward(10);
        waitforrobot(0.4);

        //*********************if this is block against the wall strafe a bit before turning
        if (BlockA) {
            strafe(-4);
            waitforrobot(0.6);
            returnto2ndstone -= 4;
        }

        //**********turn and run down the field and release block
        turneasy(-90);
        waitforrobot(0.35);
        //   checkandcorrect(-90);
        //   if(!angleok){ waitforrobot(0.6);}

        runforward((-returnto2ndstone));
        waitforrobot(0.6);
        ServoKeyStoneRed.setPosition(0.6);


        //run back to colored line
        //*********can add color sensor search if needed
        runforward(16);
        waitforrobot(0.6);


    }

    public void stopandreset() {
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runforward(double inches) {
        int delta = ((int) (inches * ticsperinch));
        telemetry.addData("delta ", delta);
        telemetry.update();
        LF.setTargetPosition(LF.getCurrentPosition() + delta);
        RF.setTargetPosition(RF.getCurrentPosition() + delta);
        RR.setTargetPosition(RR.getCurrentPosition() + delta);
        LR.setTargetPosition(LR.getCurrentPosition() + delta);
    }

    public void strafe(double inches) {
        int delta = ((int) (inches * ticsperinch * 1.1));
        LF.setTargetPosition(LF.getCurrentPosition() + delta);
        RF.setTargetPosition(RF.getCurrentPosition() - delta);
        RR.setTargetPosition(RR.getCurrentPosition() + delta);
        LR.setTargetPosition(LR.getCurrentPosition() - delta);
    }

    public void Power(double pwr1, double pwr2, double pwr3, double pwr4) {
        LF.setPower(pwr1);
        RF.setPower(pwr2);
        RR.setPower(pwr3);
        LR.setPower(pwr4);
    }

    public void runtopos() {
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runonpower() {
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public boolean robotbusy() {

        return (LF.isBusy() && RF.isBusy() && RR.isBusy() && LR.isBusy());

    }

    public boolean robotbusystrafeangle() {
        angle1 = imu.getAngularOrientation().secondAngle;
        angle2 = 0.005 * angle1 * 57.3;
        Power(0.4 - angle2, 0.4 + angle2, 0.4 + angle2, 0.4 - angle2);
        return (LF.isBusy() && RF.isBusy() && RR.isBusy() && LR.isBusy());

    }

    public boolean robotbusystrafcross(double pwr) {
        lfenccur = LF.getCurrentPosition() - lfencint;
        rfenccur = RF.getCurrentPosition() - rfencint;
        rrenccur = RR.getCurrentPosition() - rrencint;
        lrenccur = LR.getCurrentPosition() - lrencint;
        angle1 = Math.abs((lfenccur + rrenccur) / (lrenccur + rfenccur));
        if (angle1 > 0.8 && angle1 < 1.2) {
            Power(pwr, angle1 * pwr, pwr, angle1 * pwr);
        }


        return (LF.isBusy() && RF.isBusy() && RR.isBusy() && LR.isBusy());

    }

    public void waitforwardangle(double pwr, double angle) {

        err=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle-angle;
        Power(pwr, pwr, pwr, pwr);

        while (robotbusy()) {
            err = kp * (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle - angle);
            if (Math.abs(err) < 20){
                Power(pwr + err, pwr - err, pwr - err, pwr + err);

            }else{
                Power(err,-err,-err,err);}
            }
        Power(0, 0, 0, 0);
    }


    public void smallturn(double angle){

        Power(0,0,0,0);
        runonpower();

        int inc=((int) (9*angle));
        double pwr=0.35;
        int target=RF.getCurrentPosition()*inc;

        if(angle<0){
            while((RF.getCurrentPosition()-target)>0 || (tim1.seconds()-timref)<2){
                LF.setPower(.35); RF.setPower(0.35); RR.setPower(0.35); LR.setPower(0.35);
            }
            Power(0,0,0,0);
        }else{
            while((RF.getCurrentPosition()-target)<0 || (tim1.seconds()-timref)<2){
                LF.setPower(-0.35); RF.setPower(-0.35); RR.setPower(-0.35); LR.setPower(-0.35);
            }
            Power(0,0,0,0);
        }

        turn(0);
        runtopos();


    }

    public void waitforrobot(double pwr) {
        Power(pwr, pwr, pwr, pwr);
        while (robotbusy()) {
        }
        Power(0, 0, 0, 0);
    }


    public void motorencint() {
        lfencint = LF.getCurrentPosition();
        rfencint = RF.getCurrentPosition();
        rrencint = RR.getCurrentPosition();
        lrencint = LR.getCurrentPosition();

    }

    private void turneasy(double degrees) {

        int inc = ((int) (-degrees * ticsperdegree));


        LF.setTargetPosition(LF.getCurrentPosition() + inc);
        RF.setTargetPosition(RF.getCurrentPosition() - inc);
        RR.setTargetPosition(RR.getCurrentPosition() - inc);
        LR.setTargetPosition(LR.getCurrentPosition() + inc);

    }

    private void checkandcorrect(int degrees) {
        angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle;
        if (Math.abs(angle1 - degrees) > 2) {
            turneasy((degrees - angle1));
            angleok = false;
        } else {
            angleok = true;
        }

        telemetry.addData("angle out", angle1);
        telemetry.update();
        sleep(2000);
    }

    public void turn(double angle) {
        err = angle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle;
        runonpower();
        timref = tim1.seconds();
        while (Math.abs(err) > 1 && tim1.seconds() - timref < 3) {
            double lfp = kp * err;
            double rfp = -kp * err;
            double rrp = -kp * err;
            double lrp = kp * err;
            if (err < 10) {
                lfp = lfp + ki * err;
                rfp = rfp - ki * err;
                rrp = rrp - ki * err;
                lrp = lrp + ki * err;
            }
            Power(lfp, rfp, rrp, lrp);
        }

    }

    public void findimage(List<VuforiaTrackable> allTrackables) {

        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;
                sawanimage = true;


                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            avgdist = translation.get(1) / mmPerInch;
            sawanimage = true;


            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            telemetry.addData(" image ? ", sawanimage);
            telemetry.update();
        } else {

        }
        // telemetry.update();


    }


}
