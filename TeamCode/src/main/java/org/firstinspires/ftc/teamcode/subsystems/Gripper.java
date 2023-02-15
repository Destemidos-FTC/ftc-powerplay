package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.RobotConstants;

/**
 *
 */
public class Gripper implements Subsystem {
    private final ServoEx gripper;
    private final ServoEx rotator;

    /**
     *
     * @param hardwareMap
     */
    public Gripper(HardwareMap hardwareMap) {
        gripper = hardwareMap.get(ServoEx.class, "");
        rotator = hardwareMap.get(ServoEx.class, "");
    }

    /**
     *
     */
    public void closeGrip() {
        gripper.setPosition(RobotConstants.GARRA_A_POSITION_CLOSED);
    }

    /**
     *
     */
    public void releaseGrip() {
        gripper.setPosition(RobotConstants.GARRA_A_POSITION_OPEN);
    }

    /**
     *
     */
    public void rotateGripper(){
        rotator.rotateByAngle(180.0);
    }

    /**
     *
     */
    public void returnToCollectPostion() {
        rotator.rotateByAngle(0.0);
    }
}
