package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

public class ArmToGround extends SequentialCommandGroup {

    public ArmToGround(DestemidosBot robot){
        super(
                new InstantCommand(()->robot.armSystem.setForearmPosition(ArmSystem.ForearmStage.CLOSED)),
                new InstantCommand(()->robot.armSystem.setArmPosition(ArmSystem.ArmStage.CLOSED)),
                new InstantCommand(()->robot.armSystem.setForearmPosition(ArmSystem.ForearmStage.COLLECT))
        );
    }
}
