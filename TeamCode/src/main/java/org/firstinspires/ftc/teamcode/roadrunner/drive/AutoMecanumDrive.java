package org.firstinspires.ftc.teamcode.roadrunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.localizer.ThreeWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

public class AutoMecanumDrive extends com.acmerobotics.roadrunner.drive.MecanumDrive{
    private Localizer localizer;
    private final MecanumDrive drive = new MecanumDrive("drive", "Mecanum Drive");
    public AutoMecanumDrive() {
        super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, DriveConstants.TRACK_WIDTH, DriveConstants.WHEEL_RADIUS);
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        drive.init(hardwareMap);
        localizer = new ThreeWheelLocalizer(hardwareMap);
        setLocalizer(localizer);
    }

    @Override
    protected double getRawExternalHeading() {
        return 0;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();

        // TODO: Should match the four wheels used in setting the wheel power

        return wheelPositions;
    }

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();

        // TODO: Should match the four wheels used setting the wheel power

        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        drive.setMotorPowers(v, v1,v2, v3);
    }
}
