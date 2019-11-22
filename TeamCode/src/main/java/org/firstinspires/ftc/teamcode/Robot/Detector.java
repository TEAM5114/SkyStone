package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public class Detector extends VuforiaBase {
    private TFObjectDetector tfod;
    private static final String VUFORIA_KEY =
            "AXV9/0T/////AAABmdy5WsaLkE9sikFO7Jw6Eq12u+U5LoQH27GNe7jhB/Zkx+5rQBJAtxZDAmzKEXFDKnp2i8ypCE7zm8BFkg8ALmGROdE7c5A5mkc3pHm5fD8qkWAeYajXEZLUIXqUpf9aaMFR7vjqQu4QHOlA487t33Qq1GPf2rDSP94MH6AM+14Rwkf8/s2fR+g0ujNXW4lLZtiRIxdLL27b6H/GyJ66XvdijMYF8Rr2NUFo6j8X5Hm4nPV8j68s30m5bY1Ac6DDv4fJ1NPoEyhMKKmPv2YRdABoCGung9pTQXGNWg1uQIl5Ihft9Pmhohbofu3AhlhoOZHgTFX6cLV9EoWT9+BMWj0Cvrks+gpHsNBL0vcsJaGD";
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE =
            VuforiaLocalizer.CameraDirection.BACK;

    public Detector(HardwareMap hardwareMap) {
        super(hardwareMap);

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        TFObjectDetector tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, localizer);
        tfod.loadModelFromAsset("Skystone.tflite", "Stone", "Skystone");

        tfod.activate();
    }

    public void deactivate() {
        tfod.deactivate();
    }

    public int detectSkystone(double timeOut) {
        List<Recognition> recognitions = new ArrayList<>();
        ElapsedTime clock = new ElapsedTime();
        double startTime = clock.seconds();
        double elapsedTime = 0;
        while (elapsedTime < timeOut) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            recognitions = tfod.getUpdatedRecognitions();
            if (recognitions != null && recognitions.size() > 1) {
                break;
            }
            elapsedTime = clock.seconds() - startTime;
        }
        tfod.deactivate();
        if (recognitions == null) {
            return 1;
        } else if (recognitions.size() == 1) {
            if (recognitions.get(0).getLabel().equals("Skystone")) {
                return 1;
            } else {
                return 2;
            }
        } else {
            TreeMap<Float, String> treeMap = new TreeMap<>(Collections.reverseOrder());
            for (Recognition recognition : recognitions) {
                treeMap.put(recognition.getLeft(), recognition.getLabel());
            }
            int i = 1;
            for (Map.Entry<Float, String> entry : treeMap.entrySet()) {
                if (entry.getValue().equals("Skystone")) {
                    return i;
                }
                i++;
            }
            return i;
        }
    }

    public List<Recognition> detectSkystoneTest() {
        List<Recognition> recognitions = tfod.getUpdatedRecognitions();
        return recognitions;
    }
}