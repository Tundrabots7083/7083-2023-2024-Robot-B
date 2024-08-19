package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;

public interface SubsystemEx extends Subsystem {
    default void execute() {}
}
