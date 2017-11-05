package controller.ui.ui.components;

import data.states.AdvancedData;
import data.states.GamePreparationData;
import data.values.Penalties;
import data.values.Side;

import java.util.HashMap;
import java.util.Map;

/**
 * A class counting the drop in points for all players
 * Created by rkessler on 2017-05-01.
 */
public class DropInPointCounter extends AbstractComponent {

    private Map<Side, int[]> robotPoints;
    private Map<Side, Refreshable> componentMap;
    private boolean sides_as_started;

    public DropInPointCounter(){
        sides_as_started = true;
        robotPoints = new HashMap<>();
        componentMap = new HashMap<>();
    }

    public void initializeSide(Side side, int teamSize, Refreshable refreshable){
        int[] points = new int[teamSize];
        for (int i=0; i < points.length; i++){
            points[i] = 0;
        }
        robotPoints.put(side, points);
        componentMap.put(side, refreshable);
    }


    public void scoreGoal(AdvancedData data, Side teamScoring){
        if (!sides_as_started){
            teamScoring = teamScoring.getOther();
        }

        int[] pointList = robotPoints.get(teamScoring);

        for (int i = 0; i < pointList.length; i++){
            Penalties penalty = data.team[teamScoring.value()].player[i].penalty;
            int red_card_count = data.team[teamScoring.value()].player[i].redCardCount;
            if (penalty == Penalties.NONE && red_card_count == 0) {
                pointList[i] += 1;
            }
        }

        int[] pointListOther = robotPoints.get(teamScoring.getOther());
        for (int i = 0; i < pointListOther.length; i++){
            pointListOther[i] -= 1;
        }

        refreshAll();
    }

    private void refreshAll(){
        componentMap.get(Side.LEFT).refresh();
        componentMap.get(Side.RIGHT).refresh();
    }

    public int getPoints(Side side, int idx) {
        if (!sides_as_started){
            side = side.getOther();
        }
        return robotPoints.get(side)[idx];
    }

    public void deltaPoints(Side side, int idx, int delta) {
        if (!sides_as_started){
            side = side.getOther();
        }

        robotPoints.get(side)[idx] += delta;
    }

    @Override
    public void update(AdvancedData data) {
        if (this.sides_as_started != data.sides_as_started){
            this.sides_as_started = data.sides_as_started;
            refreshAll();
        }
    }
}
