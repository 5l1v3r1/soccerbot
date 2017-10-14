package controller.ui.ui.components;

import common.TotalScaleLayout;
import controller.action.ActionBoard;
import controller.action.ui.CardIncrease;
import controller.net.RobotOnlineStatus;
import controller.net.RobotWatcher;
import controller.ui.helper.FontHelper;
import controller.ui.localization.LocalizationManager;
import controller.ui.ui.customized.Button;
import controller.ui.ui.customized.CountDownBar;
import data.Helper;
import data.PlayerInfo;
import data.Rules;
import data.hl.HL;
import data.spl.SPL;
import data.states.AdvancedData;
import data.values.Penalties;
import data.values.Side;

import javax.swing.*;
import javax.swing.border.EmptyBorder;
import java.awt.*;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;

/**
 * Created by rkessler on 2017-06-11.
 */
public class RobotWithDropInCounter extends Robot implements Refreshable {

    DropInPointCounter dropInPointCounter;
    DropInStatSnippet diss;

    public RobotWithDropInCounter(Side side, int id, DropInPointCounter dropInPointCounter) {
        super(side, id);
        this.dropInPointCounter = dropInPointCounter;
    }


    public void setup(){
        diss = new DropInStatSnippet(side, id, dropInPointCounter);

        super.setup();
    }

    public void updateLayout(double aspectRatio){
        TotalScaleLayout robotLayout = new TotalScaleLayout(robot);
        robot.setLayout(robotLayout);
        robot.removeAll();

        // Figure out a way to make this easier
        double rightOffset = 0.21;

        double cardWidth = 0.4 / aspectRatio;

        robotLayout.add(1 - rightOffset + 0.02, 0.1, 0.18, 0.75, diss);

        robotLayout.add(1-cardWidth - rightOffset, 0.1, cardWidth, 0.75, yellowCard);
        robotLayout.add(1-2*cardWidth - rightOffset, 0.1, cardWidth, 0.75, redCard);

        double restWidth = 1 - 2*cardWidth - rightOffset;

        robotLayout.add(0, 0, restWidth, 0.9, robotLabel);

        robotLayout.add(0, 0.9, 1, 0.1, progressBar);
    }

    @Override
    public void refresh() {
        diss.refresh();
    }
}
