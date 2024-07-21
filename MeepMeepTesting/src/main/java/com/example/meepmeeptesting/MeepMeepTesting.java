package com.example.meepmeeptesting;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.firstinspires.ftc.teamcode.autonomous.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.trajectory.BlueBackstage;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .build();
        AutoMecanumDrive myDrive = new AutoMecanumDrive(myBot);

//        RedFrontstage trajectoryBuilder = new RedFrontstage(myDrive);
//        myBot.runAction(trajectoryBuilder.outerSpikeMarkParkCenter);
//        myBot.runAction(trajectoryBuilder.outerSpikeMarkParkBackdrop);
//        myBot.runAction(trajectoryBuilder.outerSpikeMarkParkEdge);
//        myBot.runAction(trajectoryBuilder.middleSpikeMarkParkCenter);
//        myBot.runAction(trajectoryBuilder.middleSpikeMarkParkBackdrop);
//        myBot.runAction(trajectoryBuilder.middleSpikeMarkParkEdge);
//        myBot.runAction(trajectoryBuilder.innerSpikeMarkParkCenter);
//        myBot.runAction(trajectoryBuilder.innerSpikeMarkParkBackdrop);
//        myBot.runAction(trajectoryBuilder.innerSpikeMarkParkEdge);

//        RedBackstage trajectoryBuilder = new RedBackstage(myDrive);
//        myBot.runAction(trajectoryBuilder.outerSpikeMarkParkCenter);
//        myBot.runAction(trajectoryBuilder.outerSpikeMarkParkBackdrop);
//        myBot.runAction(trajectoryBuilder.outerSpikeMarkParkEdge);
//        myBot.runAction(trajectoryBuilder.middleSpikeMarkParkCenter);
//        myBot.runAction(trajectoryBuilder.middleSpikeMarkParkBackdrop);
//        myBot.runAction(trajectoryBuilder.middleSpikeMarkParkEdge);
//        myBot.runAction(trajectoryBuilder.innerSpikeMarkParkCenter);
//        myBot.runAction(trajectoryBuilder.innerSpikeMarkParkBackdrop);
//        myBot.runAction(trajectoryBuilder.innerSpikeMarkParkEdge);

//        BlueFrontstage trajectoryBuilder = new BlueFrontstage(myDrive);
//        myBot.runAction(trajectoryBuilder.outerSpikeMarkParkCenter);
//        myBot.runAction(trajectoryBuilder.outerSpikeMarkParkBackdrop);
//        myBot.runAction(trajectoryBuilder.outerSpikeMarkParkEdge);
//        myBot.runAction(trajectoryBuilder.middleSpikeMarkParkCenter);
//        myBot.runAction(trajectoryBuilder.middleSpikeMarkParkBackdrop);
//        myBot.runAction(trajectoryBuilder.middleSpikeMarkParkEdge);
//        myBot.runAction(trajectoryBuilder.innerSpikeMarkParkCenter);
//        myBot.runAction(trajectoryBuilder.innerSpikeMarkParkBackdrop);
//        myBot.runAction(trajectoryBuilder.innerSpikeMarkParkEdge);

        BlueBackstage trajectoryBuilder = new BlueBackstage(myDrive);
//        myBot.runAction(trajectoryBuilder.outerSpikeMarkParkCenter);
//        myBot.runAction(trajectoryBuilder.outerSpikeMarkParkBackdrop);
//        myBot.runAction(trajectoryBuilder.outerSpikeMarkParkEdge);
//        myBot.runAction(trajectoryBuilder.middleSpikeMarkParkCenter);
//        myBot.runAction(trajectoryBuilder.middleSpikeMarkParkBackdrop);
//        myBot.runAction(trajectoryBuilder.middleSpikeMarkParkEdge);
        myBot.runAction(trajectoryBuilder.innerSpikeMarkParkCenter);
//        myBot.runAction(trajectoryBuilder.innerSpikeMarkParkBackdrop);
//        myBot.runAction(trajectoryBuilder.innerSpikeMarkParkEdge);

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}