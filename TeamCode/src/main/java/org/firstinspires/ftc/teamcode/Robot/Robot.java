package org.firstinspires.ftc.teamcode.Robot;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.SkyStoneDetection;

public class Robot {
    HardwareSkyStone hardware;
    MecanumDrive drive;
    SkyStoneDetection vuforia;

    float RED = 0.5f;

    public Robot(){

    }

    public void init(HardwareMap hwmap){
        hardware.init(hwmap);
        drive = new MecanumDrive(hardware);
        vuforia = new SkyStoneDetection(hardware);
    }

    public void startIMUAccelerationIntegration(){
        hardware.imu.startAccelerationIntegration(new Position(), new Velocity(), 10);
    }

    public void run(double magnitude, double heading, double pivot, boolean DRIVER_RELATIVE){
        drive.run(magnitude, heading, pivot, DRIVER_RELATIVE);
    }

    public void pivotToAngle(double power, double angle){
        drive.pivotToAngle(power, angle);
    }

    public NormalizedRGBA getColors(){
        NormalizedRGBA colors = hardware.colorSensor.getNormalizedColors();
        return colors;
    }

    public boolean colorDetected(NormalizedRGBA colors){
        return colors.red >= RED;
    }

    public Position getPosition(){
        return hardware.imu.getPosition();
    }

    public boolean skystoneIsVisible(){
        return ((VuforiaTrackableDefaultListener)vuforia.stoneTarget.getListener()).isVisible();
    }

    public OpenGLMatrix getSkystoneLocation(){
        return ((VuforiaTrackableDefaultListener)vuforia.stoneTarget.getListener()).getUpdatedRobotLocation();
    }
}
