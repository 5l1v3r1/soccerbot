package data.states;

import java.io.Serializable;

/**
 * Created by rkessler on 2017-03-23.
 */
public class GameClock implements Serializable {

    private Integer secondaryTime;
    private Long secondaryTimeStart;

    public Integer getSecondaryTime() {
        if (secondaryTimeStart == null || secondaryTime == null){
            return null;
        } else {
            return getRemainingSeconds(secondaryTimeStart, secondaryTime);
        }
    }

    public void setSecondaryClock(int time_in_seconds){
        secondaryTimeStart = System.currentTimeMillis();
        secondaryTime = time_in_seconds;
    }

    public void clearSecondaryClock(){
        secondaryTimeStart = null;
        secondaryTime = null;
    }



    public long getTime()
    {
        return System.currentTimeMillis();
    }

    public int getSecondsSince(long millis) {
        return millis == 0 ? 100000 : (int) (getTime() - millis) / 1000;
    }

    public int getRemainingSeconds(long millis, int durationInSeconds) {
        return durationInSeconds - getSecondsSince(millis);
    }

    public boolean isSecondaryClockLowerThan(Integer value) {
        Integer secondaryClock = getSecondaryTime();
        return secondaryClock != null && secondaryClock <= value;
    }

}
