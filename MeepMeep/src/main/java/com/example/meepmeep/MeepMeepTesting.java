package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
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
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 50, Math.toRadians(180), Math.toRadians(180), 11)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(30, 64, Math.toRadians(-90.00)))
                .splineTo(new Vector2d(45.19, 52.50), Math.toRadians(0.00))
                .splineTo(new Vector2d(58.07, 55.98), Math.toRadians(56.31))
                        .setReversed(true)
                .splineToSplineHeading(new Pose2d(23.07, 11.23, Math.toRadians(180)), Math.toRadians(225))
                .build());

        Image fieldBackground = null;
        try { fieldBackground = ImageIO.read(new File("/Users/rrrr/Juice-INTO-THE-DEEP-Black.png")); }
        catch(IOException ignored) {}

        meepMeep.setBackground(fieldBackground)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}