package leapmotion;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Vector3f;

public class DataTracker {
	
    HashMap<Integer, Vector3f> mFingers;
	
    public void addFinger(int id, Vector3f pos) {
        if (mFingers.get(id) == null) {
            mFingers.put(id,pos);
        }
    }

    public ArrayList<Integer> getFingerIds() {
    	ArrayList<Integer> ids = new ArrayList<Integer>();
        for (Integer i : mFingers.keySet()) {
        	ids.add(i);
        }
        return ids;
    }
    
    public void removeFinger(int id) {
    	mFingers.remove(id);
    }
    
    public Vector3f getFinger(int id) {
    	return mFingers.get(id);
    }
    
    public boolean hasFinger(int id) {
    	return mFingers.containsKey(id);
    }

}
