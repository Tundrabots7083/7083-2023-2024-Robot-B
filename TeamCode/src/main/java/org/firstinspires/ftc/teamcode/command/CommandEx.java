package org.firstinspires.ftc.teamcode.command;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;

public interface CommandEx extends Command {
    Action asAction();
}
