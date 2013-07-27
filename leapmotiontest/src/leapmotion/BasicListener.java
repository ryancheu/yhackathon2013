package leapmotion;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Set;

import com.leapmotion.leap.Controller;
import com.leapmotion.leap.Finger;
import com.leapmotion.leap.FingerList;
import com.leapmotion.leap.Frame;
import com.leapmotion.leap.Hand;
import com.leapmotion.leap.HandList;
import com.leapmotion.leap.Listener;
import com.leapmotion.leap.Vector;

public class BasicListener extends Listener {
	private FrameListener mFrameListener;
	private HashMap<Integer, FingerData> mFingerData;
	private boolean mIsOddFrame = true;
	
	public BasicListener() {
		mFingerData = new HashMap<Integer, FingerData>();
	}
	
	public interface FrameListener {
		public void onFrameGotten(float x, float y, float z);
		public void onFingerMove(int id, Vector newPosition, Vector oldPosition);
		public void onFingerNew(int id, Vector position);
		public void onFingerDelete(int id, Vector prevPosition);
	}
	
	public void setFrameListener(FrameListener fl) { 
		mFrameListener = fl;
	}
	
	
	public void onInit(Controller controller) {
		System.out.println("Initialized");
	}

	public void onConnect(Controller controller) {
		System.out.println("Connected");
	}

	public void onDisconnect(Controller controller) {
		System.out.println("Disconnected");
	}

	public void onFrame(Controller controller) {
		if (mFrameListener != null) {
			// Get the most recent frame and report some basic information
			Frame frame = controller.frame();
			HandList hands = frame.hands();
			long numHands = hands.count();
			/*System.out.println("Frame id: " + frame.id() + ", timestamp: "
					+ frame.timestamp() + ", hands: " + numHands);	*/	
			
			for (Entry<Integer, FingerData> fd : mFingerData.entrySet()) {
				fd.getValue().seenFrame = false;
			}
	
			if (numHands >= 1) {
				// Get the first hand
				
				for (Hand hand : hands) {
		
					// Check if the hand has any fingers
					FingerList fingers = hand.fingers();
					long numFingers = fingers.count();
					int[] fingersArray = new int[(int) numFingers];
					FingerData fdd;
					if (numFingers >= 1) {
						// Calculate the hand's average finger tip position
						Vector pos = new Vector(0, 0, 0);
						for (int i = 0; i < numFingers; ++i) {
							Finger finger = fingers.get(i);
							int id = finger.id();
							if (  (fdd = mFingerData.get(id)) != null) {
								fdd.seenFrame = true;
								mFrameListener.onFingerMove(id, fdd.pos, finger.tipPosition());
								fdd.pos = finger.tipPosition();
							} else {
								FingerData nFd = new FingerData();
								nFd.pos = finger.tipPosition();
								nFd.seenFrame = true;
								mFingerData.put(id, nFd);
								mFrameListener.onFingerNew(id, nFd.pos);
							}
						}
					}
		
					Vector palm = hand.palmPosition();
					Vector wrist = hand.direction();
					String direction = "";
					if (wrist.getX() > 0)
						direction = "left";
					else
						direction = "right";
					/*System.out.println("Hand is pointing to the " + direction
							+ " with palm position" + " (" + palm.getX() + ", "
							+ palm.getY() + ", " + palm.getZ() + ")");*/
				}
			}
			
			ArrayList<Integer> idsToRemove = new ArrayList<Integer> ();
			for (Entry<Integer, FingerData> fd : mFingerData.entrySet()) {
				if ( fd.getValue().seenFrame == false) {
					mFrameListener.onFingerDelete(fd.getKey(),fd.getValue().pos);
					idsToRemove.add(fd.getKey());
				}
			}
			
			for (Integer i : idsToRemove ) {
				mFingerData.remove(i);
			}
		}
			mIsOddFrame = !mIsOddFrame;
	}
}
