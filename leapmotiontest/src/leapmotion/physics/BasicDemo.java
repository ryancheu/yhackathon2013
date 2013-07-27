package leapmotion.physics;

import static com.bulletphysics.demos.opengl.IGL.GL_COLOR_BUFFER_BIT;
import static com.bulletphysics.demos.opengl.IGL.GL_DEPTH_BUFFER_BIT;

import java.io.IOException;

import javax.vecmath.Vector3f;

import leapmotion.BasicListener;
import leapmotion.BasicListener.FrameListener;

import org.lwjgl.LWJGLException;

import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.demos.opengl.DemoApplication;
import com.bulletphysics.demos.opengl.GLDebugDrawer;
import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.demos.opengl.LWJGL;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;
import com.leapmotion.leap.Controller;

/**
 * BasicDemo is good starting point for learning the code base and porting.
 * 
 * @author jezek2
 */
public class BasicDemo extends DemoApplication {

	// create 125 (5x5x5) dynamic object
	private static final int ARRAY_SIZE_X = 1;
	private static final int ARRAY_SIZE_Y = 1;
	private static final int ARRAY_SIZE_Z = 1;

	// maximum number of objects (and allow user to shoot additional boxes)
	private static final int MAX_PROXIES = (ARRAY_SIZE_X * ARRAY_SIZE_Y
			* ARRAY_SIZE_Z + 1024);

	private static final int START_POS_X = -5;
	private static final int START_POS_Y = -5;
	private static final int START_POS_Z = -3;

	// keep the collision shapes, for deletion/cleanup
	private ObjectArrayList<CollisionShape> collisionShapes = new ObjectArrayList<CollisionShape>();
	private BroadphaseInterface broadphase;
	private CollisionDispatcher dispatcher;
	private ConstraintSolver solver;
	private DefaultCollisionConfiguration collisionConfiguration;

	public BasicDemo(IGL gl) {
		super(gl);
	}

	@Override
	public void clientMoveAndDisplay() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// simple dynamics world doesn't handle fixed-time-stepping
		float ms = getDeltaTimeMicroseconds();

		// step the simulation
		if (dynamicsWorld != null) {
			dynamicsWorld.stepSimulation(ms / 1000000f);
			// optional but useful: debug drawing
			dynamicsWorld.debugDrawWorld();
		}

		renderme();

		// glFlush();
		// glutSwapBuffers();
	}

	@Override
	public void displayCallback() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		renderme();

		// optional but useful: debug drawing to detect problems
		if (dynamicsWorld != null) {
			dynamicsWorld.debugDrawWorld();
		}

		// glFlush();
		// glutSwapBuffers();
	}

	public void initPhysics() {
		setCameraDistance(50f);

		// collision configuration contains default setup for memory, collision
		// setup
		collisionConfiguration = new DefaultCollisionConfiguration();

		// use the default collision dispatcher. For parallel processing you can
		// use a diffent dispatcher (see Extras/BulletMultiThreaded)
		dispatcher = new CollisionDispatcher(collisionConfiguration);

		broadphase = new DbvtBroadphase();

		// the default constraint solver. For parallel processing you can use a
		// different solver (see Extras/BulletMultiThreaded)
		SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
		solver = sol;

		// TODO: needed for SimpleDynamicsWorld
		// sol.setSolverMode(sol.getSolverMode() &
		// ~SolverMode.SOLVER_CACHE_FRIENDLY.getMask());

		dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase,
				solver, collisionConfiguration);

		dynamicsWorld.setGravity(new Vector3f(0f, -10f, 0f));

		// create a few basic rigid bodies
		CollisionShape groundShape = new BoxShape(new Vector3f(50f, 50f, 50f));
		// CollisionShape groundShape = new StaticPlaneShape(new Vector3f(0, 1,
		// 0), 50);

		collisionShapes.add(groundShape);

		Transform groundTransform = new Transform();
		groundTransform.setIdentity();
		groundTransform.origin.set(0, -56, 0);

		// We can also use DemoApplication::localCreateRigidBody, but for
		// clarity it is provided here:
		{
			float mass = 0f;

			// rigidbody is dynamic if and only if mass is non zero, otherwise
			// static
			boolean isDynamic = (mass != 0f);

			Vector3f localInertia = new Vector3f(0, 0, 0);
			if (isDynamic) {
				groundShape.calculateLocalInertia(mass, localInertia);
			}

			// using motionstate is recommended, it provides interpolation
			// capabilities, and only synchronizes 'active' objects
			DefaultMotionState myMotionState = new DefaultMotionState(
					groundTransform);
			RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(
					mass, myMotionState, groundShape, localInertia);
			RigidBody body = new RigidBody(rbInfo);

			// add the body to the dynamics world
			dynamicsWorld.addRigidBody(body);
		}

		{
			// create a few dynamic rigidbodies
			// Re-using the same collision is better for memory usage and
			// performance

			CollisionShape colShape = new BoxShape(new Vector3f(1, 1, 1));
			// CollisionShape colShape = new SphereShape(1f);
			collisionShapes.add(colShape);

			// Create Dynamic Objects
			Transform startTransform = new Transform();
			startTransform.setIdentity();

			float mass = 1f;

			// rigidbody is dynamic if and only if mass is non zero, otherwise
			// static
			boolean isDynamic = (mass != 0f);

			Vector3f localInertia = new Vector3f(0, 0, 0);
			if (isDynamic) {
				colShape.calculateLocalInertia(mass, localInertia);
			}

			float start_x = START_POS_X - ARRAY_SIZE_X / 2;
			float start_y = START_POS_Y;
			float start_z = START_POS_Z - ARRAY_SIZE_Z / 2;

			for (int k = 0; k < ARRAY_SIZE_Y; k++) {
				for (int i = 0; i < ARRAY_SIZE_X; i++) {
					for (int j = 0; j < ARRAY_SIZE_Z; j++) {
						startTransform.origin.set(2f * i + start_x, 10f + 2f
								* k + start_y, 2f * j + start_z);

						// using motionstate is recommended, it provides
						// interpolation capabilities, and only synchronizes
						// 'active' objects
						DefaultMotionState myMotionState = new DefaultMotionState(
								startTransform);
						RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(
								mass, myMotionState, colShape, localInertia);
						RigidBody body = new RigidBody(rbInfo);
						body.setActivationState(RigidBody.ISLAND_SLEEPING);

						dynamicsWorld.addRigidBody(body);
						body.setActivationState(RigidBody.ISLAND_SLEEPING);
					}
				}
			}
		}

		clientResetScene();
	}

	public void createBox(float x, float y, float z) {
		// create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and
		// performance

		CollisionShape colShape = new BoxShape(new Vector3f(1, 1, 1));
		// CollisionShape colShape = new SphereShape(1f);
		collisionShapes.add(colShape);

		// Create Dynamic Objects
		Transform startTransform = new Transform();
		startTransform.setIdentity();

		float mass = 1f;

		// rigidbody is dynamic if and only if mass is non zero, otherwise
		// static
		boolean isDynamic = (mass != 0f);

		Vector3f localInertia = new Vector3f(0, 0, 0);
		if (isDynamic) {
			colShape.calculateLocalInertia(mass, localInertia);
		}

		float start_x = START_POS_X - ARRAY_SIZE_X / 2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ARRAY_SIZE_Z / 2;

		for (int k = 0; k < ARRAY_SIZE_Y; k++) {
			for (int i = 0; i < ARRAY_SIZE_X; i++) {
				for (int j = 0; j < ARRAY_SIZE_Z; j++) {
					startTransform.origin.set(2f * i + start_x, 10f + 2f * k
							+ start_y, 2f * j + start_z);

					// using motionstate is recommended, it provides
					// interpolation capabilities, and only synchronizes
					// 'active' objects
					DefaultMotionState myMotionState = new DefaultMotionState(
							startTransform);
					RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(
							mass, myMotionState, colShape, localInertia);
					RigidBody body = new RigidBody(rbInfo);
					body.setActivationState(RigidBody.ISLAND_SLEEPING);

					dynamicsWorld.addRigidBody(body);
					body.setActivationState(RigidBody.ISLAND_SLEEPING);
				}
			}
		}
	}
	private static Controller controller;
	private static int mNumTimeSinceLast = 0;
	public static void main(String[] args) throws LWJGLException {
		final BasicDemo ccdDemo = new BasicDemo(LWJGL.getGL());
		ccdDemo.initPhysics();
		ccdDemo.getDynamicsWorld().setDebugDrawer(
				new GLDebugDrawer(LWJGL.getGL()));

		// Create a sample listener and assign it to a controller to receive
		// events
		BasicListener listener = new BasicListener();
		
		listener.setFrameListener(new FrameListener() {
			
			@Override
			public void onFrameGotten(float x, float y, float z) {
				if (mNumTimeSinceLast++ % 100 ==0 ) {
					ccdDemo.createBox(x,y,z);
				}
			}
		});
		controller = new Controller(listener);
		
		
		// The controller must be disposed of before the listener

		LWJGL.main(args, 800, 600, "Bullet Physics Demo. http://bullet.sf.net",
				ccdDemo);
	}

}