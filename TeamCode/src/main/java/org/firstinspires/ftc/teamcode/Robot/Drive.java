package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Arrays;
import java.util.List;

public class Drive {
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor rightFront;
    private List<DcMotor> motors;
    private BNO055IMU imu;

    private double WHEEL_DIAMETER = 4;
    private double ticsperinch = 480.0 / WHEEL_DIAMETER / Math.PI;

    private double startHeading = 0;

    public Drive(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotor motor : motors){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

    }

    public void forwardToPosition(double inches, double power) {
        int delta = ((int) (inches * ticsperinch));
//        telemetry.addData("delta ", delta);
//        telemetry.update();
        for (DcMotor motor : motors){
            motor.setTargetPosition(motor.getCurrentPosition() + delta);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        setPower(power, 0, 0);
        while (isBusy()) {
        }
        setPower(0,0,0);
    }

    public void strafeToPosition(double inches, double power) {
        int delta = ((int) (inches * ticsperinch * 1.1));
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + delta);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() - delta);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + delta);
        rightFront.setTargetPosition(leftFront.getCurrentPosition() - delta);

        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        setPower(0, power, 0);
        while (isBusy()) {
        }
        setPower(0,0,0);
    }

    public void turnThruAngle(double angle, double power){
        power *= Math.signum(angle);
        double target = getCurrentHeading() + angle;
        turnToHeading(target, power);
    }

    public void turnToHeading(double targetHeading, double power){
        if (Math.abs(targetHeading) > 180){
            targetHeading = (targetHeading % 180) + (-1 * Math.signum(targetHeading) * 180);
        }
        double angle = Math.abs(getCurrentHeading() - targetHeading);
        double err = Math.min(angle, (360 - angle));
        while (err > 2){
            setPower(0,0, power);
            angle = Math.abs(getCurrentHeading() - targetHeading);
            err = Math.min(angle, (360 - angle));
        }
        setPower(0,0,0);
    }

    public void driveWithoutEncoders(double forward, double left, double turn){
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        setPower(forward, left, turn);
    }

    public void setPower(double forward, double left, double turn){
        double leftFrontPower = forward - left - turn;
        double leftRearPower = forward + left - turn;
        double rightRearPower = forward - left + turn;
        double rightFrontPower = forward + left + turn;

        double maxPower = Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower)),
                Math.max(Math.abs(rightRearPower), Math.abs(rightFrontPower)));

        if (maxPower > 1) {
            leftFrontPower /= maxPower;
            leftRearPower /= maxPower;
            rightRearPower /= maxPower;
            rightFrontPower /= maxPower;
        }
        setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
    }

    private void setMotorPowers(double p1, double p2, double p3, double p4) {
        leftFront.setPower(p1);
        leftRear.setPower(p2);
        rightRear.setPower(p3);
        rightFront.setPower(p4);
    }

    private boolean isBusy(){
        for (DcMotor motor : motors){
            if (motor.isBusy()){
                return true;
            }
        }
        return false;
    }

    public double getStartHeading() {
        return startHeading;
    }

    public void setStartHeading(double startHeading) {
        this.startHeading = startHeading;
    }

    public double getCurrentHeading(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + startHeading;
    }
}
