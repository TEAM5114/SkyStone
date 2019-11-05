package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryConfig;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.fasterxml.jackson.module.kotlin.KotlinModule;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.IOException;
import java.io.InputStream;

/**
 * Utilities to load trajectories generated from the trajectory builder plugin (saved in assets)
 */

public class AssetsTrajectoryLoader {
    private static final ObjectMapper MAPPER = new ObjectMapper(new YAMLFactory());

    static {
        MAPPER.registerModule(new KotlinModule());
    }

    /**
     * Load a trajectory config by name
     */
    public static TrajectoryConfig loadConfig(String name) throws IOException {
        InputStream inputStream = AppUtil.getDefContext().getAssets().open("trajectory/" + name + ".yaml");
        return MAPPER.readValue(inputStream, TrajectoryConfig.class);
    }

    /**
     * Load a trajectory with a given name
     * @see #loadConfig(String)
     */
    public static Trajectory load(String name) throws IOException {
        return loadConfig(name).toTrajectory();
    }
}
