package org.firstinspires.ftc.teamcode.libs;

import java.util.LinkedList;

// runningVoteCount : A utility class for keeping track of of running totals of "votes" over
// the last N seconds.

public class runningVoteCount {
    private int[] totals;
    private long timeWindow;
    private class voteWrapper {
        public int vote;
        public long sampleTime;
        voteWrapper(int v, long t){
            vote = v;
            sampleTime = t;
        }
    }
    private LinkedList<voteWrapper> voteQueue;


    // milliSeconds is the amount of time the vote should "look back"
    public runningVoteCount(int milliSeconds) {
        voteQueue = new LinkedList<>();
        timeWindow = milliSeconds;
        totals = new int[4];
        totals[1] = 0;
        totals[2] = 0;
        totals[3] = 0;
    }

    public void clearVote() {
        voteQueue.clear();
        totals[1] = 0;
        totals[2] = 0;
        totals[3] = 0;

    }
    // Cast a vote (1, 2, or 3)
    public void vote (int vote) {
        // keep a running list of samples (and their total) that have been taken in the last timeWindow milliseconds
        long now = System.currentTimeMillis();
        long cutoff = now - timeWindow;
        totals[vote]= totals[vote]+1; // add the new vote to our totals
        voteQueue.addFirst(new voteWrapper(vote, now)); // add the new vote to the list
        while (voteQueue.getLast().sampleTime<cutoff) { // remove stale votes from end of list
            int removedVote = voteQueue.removeLast().vote;
            totals[removedVote]= totals[removedVote]-1; // remove stale vote from our totals
        }
    }

    // return the candidate (1, 2 or 3) that has the most votes over the most recent timewindow
    // threshold is the factor the winner must have over second place
    public int getWinner(float threshold){
        if (totals[1] > totals[2]*threshold && totals[1]> totals[3]*threshold) return 1; else
        if (totals[2] > totals[1]*threshold && totals[2]> totals[3]*threshold) return 2; else
        if (totals[3] > totals[1]*threshold && totals[3]> totals[2]*threshold) return 3; else
            return 0;
    }
    // get all the totals over the most recent timewindow (for diagnostic purposes)
    public int[] getTotals () { return totals;}
}

