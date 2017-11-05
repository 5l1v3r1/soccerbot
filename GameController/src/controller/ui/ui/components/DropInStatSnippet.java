package controller.ui.ui.components;

import common.TotalScaleLayout;
import controller.action.ActionBoard;
import controller.action.ActionType;
import controller.action.GCAction;
import controller.net.RobotOnlineStatus;
import controller.net.RobotWatcher;
import controller.ui.ui.customized.Button;
import data.Rules;
import data.communication.GameControlData;
import data.hl.HL;
import data.spl.SPL;
import data.states.AdvancedData;
import data.values.GameStates;
import data.values.Penalties;
import data.values.SecondaryGameStates;
import data.values.Side;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

/**
 * A small component counting drop in points
 * Can be added to the robot label if necessary
 * Created by rkessler on 2017-03-29.
 */
public class DropInStatSnippet extends JLabel implements Refreshable {

    private Side side;
    private int robotId;
    private DropInPointCounter dropInPointCounter;

    private JButton robotPointsInc;
    private JButton robotPointsDec;
    private JLabel robotPointsLabel;

    public DropInStatSnippet(Side side, int robotId, DropInPointCounter dropInPointCounter){
        this.side = side;
        this.robotId = robotId;
        this.dropInPointCounter = dropInPointCounter;

        robotPointsInc = new Button("+");
        robotPointsDec = new Button("-");
        robotPointsLabel = new JLabel("0 Points");

        robotPointsInc.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent actionEvent) {
                dropInPointCounter.deltaPoints(side, robotId, 1);
                updateDropInPointsLabels();
            }
        });

        robotPointsDec.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent actionEvent) {
                dropInPointCounter.deltaPoints(side, robotId, -1);
                updateDropInPointsLabels();
            }
        });

        defineLayout();
    }

    private void updateDropInPointsLabels(){
        int points = dropInPointCounter.getPoints(side, robotId);
        robotPointsLabel.setText(points + " Points");
    }


    public void defineLayout(){
        TotalScaleLayout layout = new TotalScaleLayout(this);
        this.setLayout(layout);

        layout.add(0, 0, 1, 0.35, robotPointsInc);
        layout.add(0, 0.35, 1, 0.35, robotPointsDec);
        layout.add(0, 0.70, 1, 0.3, robotPointsLabel);
    }

    @Override
    public void refresh() {
        updateDropInPointsLabels();
    }
}
