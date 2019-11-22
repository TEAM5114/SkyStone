package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class VuforiaBase {
    WebcamName webcamName = null;
    private static final String VUFORIA_KEY =
            "ARxlubj/////AAABmaAWvvNcaEfatpedWzcPYoNNzXvozddqiaVF0eqgKR71EFvFsUChMN1cTJhTfx7pLvbxNnz338HqcYkMq3vkWg+AWEGUPcomz44FQ6ax2INPKro2ZXiUasw3qjx6yDi7TyD0b5zW8MVBBDJrqlEmCi13JLVWZTtmrG4xAh1LryGISe2tT9fqHV04K2nuynDDRGcT9O3WT4yghlNQ/vWnR3oJkgidaGUCsCV6UqfrLAa94r9Lkd/bSyvr29q8qkS5HXJSjFmlXMabP3mTVNhX2e1mfwm8AGjMRBzJPtyqBp7OJUqADQgP5mDH+Qs4gPeKrtcM1FfNurJyxe4MTaumoJYMoRa6PoY9jlkAlITh5kio";
    VuforiaLocalizer localizer = null;
    VuforiaLocalizer.Parameters parameters;


    public VuforiaBase(HardwareMap hardwareMap) {

        webcamName = hardwareMap.get(WebcamName.class, "webcam");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        localizer = ClassFactory.getInstance().createVuforia(parameters);
    }
}
