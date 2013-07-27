package leapmotion;

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
	
	public interface FrameListener {
		public void onFrameGotten(float x, float y, float z);
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
		// Get the most recent frame and report some basic information
		Frame frame = controller.frame();
		HandList hands = frame.hands();
		long numHands = hands.count();
		System.out.println("Frame id: " + frame.id() + ", timestamp: "
				+ frame.timestamp() + ", hands: " + numHands);

		if (numHands >= 1) {
			// Get the first hand
			Hand hand = hands.get(0);

			// Check if the hand has any fingers
			FingerList fingers = hand.fingers();
			long numFingers = fingers.count();
			if (numFingers >= 1) {
				// Calculate the hand's average finger tip position
				Vector pos = new Vector(0, 0, 0);
				for (int i = 0; i < numFingers; ++i) {
					Finger finger = fingers.get(i);
					Vector tip = finger.tipPosition();
					pos.setX(pos.getX() + tip.getX());
					pos.setY(pos.getY() + tip.getY());
					pos.setZ(pos.getZ() + tip.getZ());
				}
				pos = new Vector(pos.getX() / numFingers, pos.getY()
						/ numFingers, pos.getZ() / numFingers);
				System.out.println("Hand has " + numFingers
						+ " fingers with average tip position" + " ("
						+ pos.getX() + ", " + pos.getY() + ", " + pos.getZ()
						+ ")");
				if (mFrameListener != null) {
					mFrameListener.onFrameGotten(pos.getX(), pos.getY(), pos.getZ());
				}
			}

			Vector palm = hand.palmPosition();
			Vector wrist = hand.direction();
			String direction = "";
			if (wrist.getX() > 0)
				direction = "left";
			else
				direction = "right";
			System.out.println("Hand is pointing to the " + direction
					+ " with palm position" + " (" + palm.getX() + ", "
					+ palm.getY() + ", " + palm.getZ() + ")");
		}
	}
}
