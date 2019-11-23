package org.firstinspires.ftc.teamcode.Drive;


import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public class SkystoneDetector {
    private TFObjectDetector tfod;
    private NanoClock clock = NanoClock.system();


    public SkystoneDetector(TFObjectDetector tfod) {
        this.tfod = tfod;
        tfod.activate();
    }

    public void deactivate(){
        tfod.deactivate();
    }

    public int detectSkystone(double timeOut){
        List<Recognition> recognitions = new ArrayList<>();
        double startTime = clock.seconds();
        double elapsedTime = 0;
        while (elapsedTime < timeOut) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            recognitions = tfod.getUpdatedRecognitions();
            if (recognitions != null && recognitions.size() > 1){ break; }
            elapsedTime = clock.seconds() - startTime;
        }
        tfod.deactivate();
        if (recognitions == null) {
            return 1;
        } else if (recognitions.size() == 1){
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

    public List<Recognition> detectSkystoneTest(){
        List<Recognition> recognitions = tfod.getUpdatedRecognitions();
        return recognitions;
    }
}
