/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Motor channel:  Left front drive motor:        "left_front_drive"
 * Motor channel:  Left rear drive motor:        "left_rear_drive"
 * Motor channel:  Right front drive motor:        "right_front_drive"
 * Motor channel:  Right rear drive motor:        "right_rear_drive"
 */
public class HardwareSkyStone
{
    /* Public OpMode members. */
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  leftRearDrive   = null;
    public DcMotor  rightFrontDrive   = null;
    public DcMotor  rightRearDrive   = null;
    public BNO055IMU imu;
    public NormalizedColorSensor colorSensor;
    public int cameraMonitorViewID;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true;
    private static final String VUFORIA_KEY =
            "AXV9/0T/////AAABmdy5WsaLkE9sikFO7Jw6Eq12u+U5LoQH27GNe7jhB/Zkx+5rQBJAtxZDAmzKEXFDKnp2i8ypCE7zm8BFkg8ALmGROdE7c5A5mkc3pHm5fD8qkWAeYajXEZLUIXqUpf9aaMFR7vjqQu4QHOlA487t33Qq1GPf2rDSP94MH6AM+14Rwkf8/s2fR+g0ujNXW4lLZtiRIxdLL27b6H/GyJ66XvdijMYF8Rr2NUFo6j8X5Hm4nPV8j68s30m5bY1Ac6DDv4fJ1NPoEyhMKKmPv2YRdABoCGung9pTQXGNWg1uQIl5Ihft9Pmhohbofu3AhlhoOZHgTFX6cLV9EoWT9+BMWj0Cvrks+gpHsNBL0vcsJaGD";
    public VuforiaLocalizer vuforia = null;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;
    private static final float mmPerInch = 25.4f;
    public VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewID);
    public OpenGLMatrix robotFromCamera;
    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public HardwareSkyStone(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive = hwMap.get(DcMotor.class, "left_rear_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        rightRearDrive = hwMap.get(DcMotor.class, "right_rear_drive");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Initialize Camera for vuforia
        cameraMonitorViewID = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewID", "id", hwMap.appContext.getPackageName());

        vuforiaParameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforiaParameters.cameraDirection = CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);

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

        robotFromCamera = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        // Initialize Color Sensor
        colorSensor = hwMap.get(NormalizedColorSensor.class, "sensor_color");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
        // Define and initialize ALL installed servos.
    }
 }

