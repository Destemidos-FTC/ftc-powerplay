package org.firstinspires.ftc.teamcode.test;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

@TeleOp(name = "TestServo", group = "Test")
public class TestServo extends CommandOpMode {
    private Gripper gripper;

    @Override
    public void initialize() {
        gripper = new Gripper(hardwareMap);

        CommandScheduler.getInstance().reset();
        register(gripper);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        if(gamepad1.x) {
            gripper.closeGrip();
        }

        if (gamepad1.y) {
            gripper.releaseGrip();
        }

        if(gamepad1.right_bumper) {
            gripper.moveWrist(1);
        }

        if (gamepad1.left_bumper) {
            gripper.moveWrist(-1);
        }

        telemetry.addData("gripper position", gripper.gripper.getPosition());
        telemetry.addData("wristServoA power", gripper.wristServoA.getPower());
        telemetry.addData("wristServoB power", gripper.wristServoB.getPower());
        telemetry.update();
    }
}