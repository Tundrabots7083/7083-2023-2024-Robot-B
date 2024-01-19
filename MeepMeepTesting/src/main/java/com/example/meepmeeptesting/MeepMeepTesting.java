package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Declare our first bot
//        RoadRunnerBotEntity redAllianceBackstage = new DefaultBotBuilder(meepMeep)
//                // We set this bot to be blue
//                .setColorScheme(new ColorSchemeBlueDark())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
//                                .splineToLinearHeading(new Pose2d(-36,-34), Math.toRadians(0))
//                                .addDisplacementMarker(() -> System.out.println("Drop purple pixel"))
//                                .back(10)
//                                .splineTo(new Vector2d(-57, -40), Math.toRadians(90))
//                                .back(10)
//                                .splineTo(new Vector2d(0, -10), 0)
//                                .back(23)
//                                .splineTo(new Vector2d(48, -30), 0)
//                                .addDisplacementMarker(() -> System.out.println("Score yellow pixel"))
//                                .strafeRight(19)
//                                .back(5)
//                                .addDisplacementMarker(() -> System.out.println("Park robot\n"))
//                                .back(1) // Unnecessary, but makes the display look better
//                                .build()
//                );
        RoadRunnerBotEntity redAllianceBackstage = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
                                .splineToLinearHeading(new Pose2d(-36,-34), Math.toRadians(0))
                                .addDisplacementMarker(() -> System.out.println("Drop purple pixel"))
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(-35, -58, 0), Math.toRadians(-90))
                                .build()
                );

//        // Declare our first bot
//        RoadRunnerBotEntity redAllianceFrontstage = new DefaultBotBuilder(meepMeep)
//                // We set this bot to be blue
//                .setColorScheme(new ColorSchemeBlueDark())
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

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(redAllianceBackstage)
//                .addEntity(redAllianceFrontstage)
                .start();
    }
}