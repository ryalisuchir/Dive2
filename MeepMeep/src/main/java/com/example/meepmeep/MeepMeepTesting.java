package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(24.46, 60.51, Math.toRadians(0.00)))
                .splineTo(new Vector2d(53.37, 53.37), Math.toRadians(39.81))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(24, 4.96, Math.toRadians(0)), Math.toRadians(180))
                .build());

        Image fieldBackground = null;
        try { fieldBackground = ImageIO.read(new File("/Users/rrrr/Downloads/Juice-INTO-THE-DEEP-Light.png")); }
        catch(IOException ignored) {}

        meepMeep.setBackground(fieldBackground)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}