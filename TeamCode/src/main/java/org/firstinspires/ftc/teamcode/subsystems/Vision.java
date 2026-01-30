package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Vision {
    private final HuskyLens huskyLens;
    private HuskyLens.Block[] currentBlocks;

    // Known AprilTag height in inches (Standard 360 size is ~2.0 inches)
    private static final double TAG_HEIGHT_INCHES = 6.5;
    
    // --- Focal Length Calibration ---
    // Physical focal length of the lens (front focal length is typically used for the pinhole model)
    private static final double LENS_FOCAL_LENGTH_MM = 4.6; 
    // Sensor height for a typical HuskyLens sensor (e.g., OV2640 1/4" sensor height is ~2.7mm)
    private static final double SENSOR_HEIGHT_MM = 2.7;
    // Resolution height in pixels (HuskyLens default is 240)
    private static final int IMAGE_HEIGHT_PIXELS = 240;

    /**
     * Calculated Pixel Focal Length (F_pixel)
     * formula: F_pixel = (f_mm * ImageHeight_pixels) / SensorHeight_mm
     */
    private static final double FOCAL_LENGTH_PIXELS = (LENS_FOCAL_LENGTH_MM * IMAGE_HEIGHT_PIXELS) / SENSOR_HEIGHT_MM;

    public Vision(HardwareMap hardwareMap, String name) {
        huskyLens = hardwareMap.get(HuskyLens.class, name);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    public void update() {
        currentBlocks = huskyLens.blocks();
    }

    public int getTargetTagX(int targetId) {
        HuskyLens.Block block = getBlockById(targetId);
        return (block != null) ? block.x : -1;
    }

    /**
     * Estimates distance to an AprilTag based on its height in the frame.
     * Uses the calibrated focal length derived from physical lens parameters.
     * @param targetId The ID of the tag.
     * @return Distance in inches, or -1 if not found.
     */
    public double getDistanceToTag(int targetId) {
        HuskyLens.Block block = getBlockById(targetId);
        if (block == null || block.height == 0) return -1;

        // distance = (F_pixel * RealHeight) / PixelHeight
        return (FOCAL_LENGTH_PIXELS * TAG_HEIGHT_INCHES) / block.height;
    }

    private HuskyLens.Block getBlockById(int targetId) {
        if (currentBlocks == null) return null;
        for (HuskyLens.Block block : currentBlocks) {
            if (block.id == targetId) return block;
        }
        return null;
    }

    public HuskyLens.Block[] getCurrentBlocks() {
        return currentBlocks;
    }
}
