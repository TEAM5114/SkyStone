package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {
    public DcMotor LF;
    public DcMotor RF;
    public DcMotor LR;
    public DcMotor RR;
    public BNO055IMU imu;

    private double ticsperinch = 480 / 4 / 3.1415926;
    private double ticsperdegree = 8.6444444444;

    private double angle1 = 0;
    private double angle2 = 0;
    private int lfencint = 0;
    private int rfencint = 0;
    private int rrencint = 0;
    private int lrencint = 0;
    private int lfenccur = 0;
    private int rfenccur = 0;
    private int rrenccur = 0;
    private int lrenccur = 0;



    public Drive(HardwareMap hardwareMap) {
        LF = hardwareMap.get(DcMotor.class, "lf");
        RF = hardwareMap.get(DcMotor.class, "rf");
        LR = hardwareMap.get(DcMotor.class, "lr");
        RR = hardwareMap.get(DcMotor.class, "rr");

        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        LR.setDirection(DcMotorSimple.Direction.FORWARD);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    }

    public void forwardToPosition(double inches, double power) {
        int delta = ((int) (inches * ticsperinch));
//        telemetry.addData("delta ", delta);
//        telemetry.update();
        LF.setTargetPosition(LF.getCurrentPosition() + delta);
        RF.setTargetPosition(RF.getCurrentPosition() + delta);
        RR.setTargetPosition(RR.getCurrentPosition() + delta);
        LR.setTargetPosition(LR.getCurrentPosition() + delta);

        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        power(power);
    }

    public void forwardToPositionWait(double inches, double power){
        forwardToPosition(inches, power);
        while (isBusy()){
            continue;
        }
        power(0);
    }

    public void strafeToPosition(double inches, double power) {
        int delta = ((int) (inches * ticsperinch * 1.1));
        LF.setTargetPosition(LF.getCurrentPosition() + delta);
        RF.setTargetPosition(RF.getCurrentPosition() - delta);
        RR.setTargetPosition(RR.getCurrentPosition() + delta);
        LR.setTargetPosition(LR.getCurrentPosition() - delta);

        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        power(power);
    }

    public void  strafeToPositionWait(double inches, double power){
        strafeToPosition(inches, power);
        while (isBusy()){
            continue;
        }
        power(0);
    }

    public void turnThruAngle(double angle, double power){
        int inc = ((int) (-angle * ticsperdegree));

        LF.setTargetPosition(LF.getCurrentPosition() + inc);
        RF.setTargetPosition(RF.getCurrentPosition() - inc);
        RR.setTargetPosition(RR.getCurrentPosition() - inc);
        LR.setTargetPosition(LR.getCurrentPosition() + inc);

        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        power(power);
    }

    public void turnThruAngleWait(double angle, double power){
        turnThruAngle(angle, power);
        while (isBusy()){
            continue;
        }
        power(0);
    }

    public boolean isBusy(){
        return LF.isBusy() || RF.isBusy() || LR.isBusy() || RR.isBusy();
    }

    public void power(double power){
        LF.setPower(power);
        RF.setPower(power);
        LR.setPower(power);
        RR.setPower(power);
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






}
