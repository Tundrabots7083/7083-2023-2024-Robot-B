package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {


    public static final Pose2d BACK_STAGE_START_POSITION = new Pose2d(12, 63.5, Math.toRadians(-90));

    public static final Pose2d BACK_STAGE_INNER_SPIKE_BASE = new Pose2d(12, 40, Math.toRadians(-135));
    public static final Pose2d BACK_STAGE_MIDDLE_SPIKE_BASE = new Pose2d(12, 40, Math.toRadians(-90));
    public static final Pose2d BACK_STAGE_OUTER_SPIKE_BASE = new Pose2d(23, 40, Math.toRadians(-90));

    public static final int INNER_SPIKE_BASE_HEADING = 180;

    public static final int MIDDLE_SPIKE_BASE_HEADING = -90;

    public static final int OUTER_SPIKE_BASE_HEADING = -90;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

//        RoadRunnerBotEntity redAllianceLeft = new DefaultBotBuilder(meepMeep)
//                // We set this bot to be blue
//                .setColorScheme(new ColorSchemeBlueDark())
//                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 14)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
//                                .splineToLinearHeading(new Pose2d(-36, -34), Math.toRadians(0))
//                                .addDisplacementMarker(() -> System.out.println("Drop purple pixel"))
//                                .setTangent(Math.toRadians(-90))
//                                .splineToLinearHeading(new Pose2d(-35, -58, Math.toRadians(180)), Math.toRadians(0))
//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(35, -58, Math.toRadians(180)), Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(49, -36), Math.toRadians(0))
//                                .build()
//                );
//
//        RoadRunnerBotEntity redAllianceRight = new DefaultBotBuilder(meepMeep)
//                .setColorScheme(new ColorSchemeRedDark())
//                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 14)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(12, -63.5, Math.toRadians(90)))
//                                .lineTo(new Vector2d(12, -43.5))
//                                .addDisplacementMarker(() -> System.out.println("Drop purple pixel"))
//                                .lineTo(new Vector2d(12, -48.5))
//                                .lineTo(new Vector2d(24, -48.5))
//                                .splineTo(new Vector2d(48, -35), Math.toRadians(90))
//                                .addDisplacementMarker(() -> System.out.println("Score yellow pixel"))
//                                .lineTo(new Vector2d(48, -59))
//                                .lineTo(new Vector2d(58, -59))
//                                .addDisplacementMarker(() -> System.out.println("Park robot\n"))
//                                .back(1) // Unnecessary, but makes the display look better
//                                .build()
//                );

        BlueBackstageTrajectoryBuilder blueBackstageTrajectoryBuilder = new BlueBackstageTrajectoryBuilder();

        RoadRunnerBotEntity redAllianceRight = new DefaultBotBuilder(meepMeep)
            .setColorScheme(new ColorSchemeRedDark())
            .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 14)
            .followTrajectorySequence(drive ->
                    blueBackstageTrajectoryBuilder.getTrajectory(TeamElementLocation.MIDDLE, drive)
            );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
//                .addEntity(redAllianceLeft)
                .addEntity(redAllianceRight)
                .start();
    }
}