/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

/*
 * CO-PROCESSOR USED:           Raspberry Pi Model 3B+ w/ PhotonVision
 * CAMERA USED:                 Microsoft LifeCam HD-3000
 * MAX RESOLUTION:              1280x720 @ 30FPS
 * 
 * APRILTAG DETECTION:          352x288 @ 7FPS, MJPEG
 * REFLECTIVE TAPE DETECTION:   N/A
 */

package com.sentinels.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera extends SubsystemBase {

    // Initializing camera object for later use in this file
    PhotonCamera camera = new PhotonCamera("photonvision");

    public Camera() {
        
    }

    /**
    * Example command factory method.
    *
    * @return a command
    */
    public CommandBase exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
            /* one-time action goes here */
        });
    }

    /**
    * Query the state of target detection.
    *
    * @return {@code true} if a target has been detected, {@code false} if nothing has been detected
    */
    public boolean targetDetected() {

        // Storing latest camera pipeline result in var variable.
        var result = camera.getLatestResult();
        // Checking for targets, stores true if detected, false if not.
        boolean hasTargets = result.hasTargets();

        if (hasTargets) {
            return true;
        }
        else {
            return false;
        }
    }
    /**
     * Query list of all detected targets.
     * 
     * @return List of information about target(s) including yaw, pitch, area, and robot relative pose.
     */
    public List<PhotonTrackedTarget> getAllTargets() {

        // Storing latest camera pipeline result in var variable.
        var result = camera.getLatestResult();

        return result.getTargets();
    }
    /**
     * Query information on best detected target.
     * 
     * @return Target information.
     */
    public PhotonTrackedTarget getBestTarget() {

        // Storing latest camera pipeline result in var variable.
        var result = camera.getLatestResult();

        return result.getBestTarget();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
        public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}