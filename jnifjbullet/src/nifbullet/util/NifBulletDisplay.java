package nifbullet.util;

import java.io.File;
import java.util.ArrayList;
import java.util.prefs.Preferences;

import javax.media.j3d.BranchGroup;
import javax.swing.filechooser.FileNameExtensionFilter;
import javax.vecmath.Vector3f;

import nativeLinker.LWJGLLinker;
import nif.gui.NifKfFileFilter;
import nif.gui.util.ControllerInvokerThread;
import nifbullet.BulletNifModel;
import nifbullet.BulletNifModelClassifier;
import nifbullet.simple.NBSimpleModel;
import nifbullet.util.debug.opengl.DebugOutput;
import nifbullet.util.debug.opengl.LWJGL;
import tools.swing.DetailsFileChooser;
import tools3d.universe.HeadlessUniverse;
import utils.source.file.FileMeshSource;

import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.Clock;

public class NifBulletDisplay
{

	private HeadlessUniverse j3dUni = new HeadlessUniverse();

	// Gravity
	private static Vector3f gravity = new Vector3f(0f, 0f, -9.81f);

	protected static Clock clock = new Clock();

	// this is the most important class
	protected static DynamicsWorld dynamicsWorld = null;

	private static BroadphaseInterface broadphase;

	private static CollisionDispatcher dispatcher;

	private static ConstraintSolver solver;

	private static DefaultCollisionConfiguration collisionConfiguration;

	private static Preferences prefs;

	public static NifBulletDisplay nifDisplay;

	private long currentFileLoadTime = 0;

	private File currentFileTreeRoot;

	private File nextFileTreeRoot;

	private File currentFileDisplayed;

	private File nextFileToDisplay;

	public void setNextFileTreeRoot(File nextFileTreeRoot)
	{
		this.nextFileToDisplay = null;
		this.nextFileTreeRoot = nextFileTreeRoot;
	}

	public void setNextFileToDisplay(File nextFileToDisplay)
	{
		this.nextFileTreeRoot = null;
		this.nextFileToDisplay = nextFileToDisplay;
	}

	public void manage()
	{
		if (nextFileTreeRoot != null)
		{
			if (!nextFileTreeRoot.equals(currentFileTreeRoot))
			{
				currentFileTreeRoot = nextFileTreeRoot;
				currentFileDisplayed = null;
				currentFileLoadTime = Long.MAX_VALUE;
			}
		}
		else if (currentFileTreeRoot != null)
		{

			File[] files = currentFileTreeRoot.listFiles(new NifKfFileFilter());
			if (files.length > 0)
			{
				if (currentFileDisplayed == null)
				{
					currentFileDisplayed = files[0];
					displayNif(currentFileDisplayed);
				}
				else if (System.currentTimeMillis() - currentFileLoadTime > 3000)
				{

				}
			}

		}
		else if (nextFileToDisplay != null)
		{
			if (!nextFileToDisplay.equals(currentFileDisplayed))
			{
				currentFileDisplayed = nextFileToDisplay;
				displayNif(currentFileDisplayed);
				nextFileToDisplay = null;
			}
		}
	}

	public void displayNif(File f)
	{
		System.out.println("Selected file: " + f);

		if (f.isDirectory())
		{
			//spinTransform.setEnable(true);
			processDir(f);
			System.out.println("Bad news dir sent into display nif");
		}
		else if (f.isFile())
		{
			try
			{
				for (BulletNifModel bnm : bulletNifModels)
				{
					bnm.removeFromDynamicsWorld();
				}

				BulletNifModel nb = BulletNifModelClassifier.createNifBullet(f.getAbsolutePath(), new FileMeshSource(), 0);
				if (nb != null)
				{
					nb.addToDynamicsWorld(dynamicsWorld);
					bulletNifModels.add(nb);
					startDisplay();

					if (nb instanceof NBSimpleModel)
					{
						NBSimpleModel nbKinematicModel = (NBSimpleModel) nb;
						if (nbKinematicModel.getJ3dNiControllerManager() != null)
						{
							BranchGroup bg = new BranchGroup();
							bg.addChild(nbKinematicModel);
							j3dUni.addBranchGraph(bg);
							//note self cleaning uping
							ControllerInvokerThread controllerInvokerThread = new ControllerInvokerThread(f.getAbsolutePath(),
									nbKinematicModel.getJ3dNiControllerManager(), null);
							controllerInvokerThread.start();
						}
					}

				}
			}
			catch (Exception ex)
			{
				ex.printStackTrace();
			}
		}

		System.out.println("done");

	}

	public static void main(String[] args)
	{

		new LWJGLLinker();

		nifDisplay = new NifBulletDisplay();
		prefs = Preferences.userNodeForPackage(NifBulletDisplay.class);
		String baseDir = prefs.get("NifBulletDisplay.baseDir", System.getProperty("user.dir"));

		DetailsFileChooser dfc = new DetailsFileChooser(baseDir, new DetailsFileChooser.Listener()
		{

			@Override
			public void directorySelected(File dir)
			{
				prefs.put("NifBulletDisplay.baseDir", dir.getPath());
				nifDisplay.setNextFileTreeRoot(dir);
			}

			@Override
			public void fileSelected(File file)
			{
				prefs.put("NifBulletDisplay.baseDir", file.getPath());
				nifDisplay.setNextFileToDisplay(file);
			}
		});

		dfc.setFileFilter(new FileNameExtensionFilter("Nif or Kf", "nif", "kf"));

		// collision configuration contains default setup for memory, collision setup
		collisionConfiguration = new DefaultCollisionConfiguration();

		// use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		dispatcher = new CollisionDispatcher(collisionConfiguration);

		broadphase = new DbvtBroadphase();

		// the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
		SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
		solver = sol;

		// TODO: needed for SimpleDynamicsWorld
		//sol.setSolverMode(sol.getSolverMode() & ~SolverMode.SOLVER_CACHE_FRIENDLY.getMask());

		dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

		dynamicsWorld.setGravity(gravity);

		Thread t = new Thread(new Runnable()
		{

			@Override
			public void run()
			{
				while (true)
				{
					try
					{
						Thread.sleep(100);
					}
					catch (Exception e)
					{
						e.printStackTrace();
					}
					nifDisplay.manage();
				}
			}
		});
		t.setDaemon(true);
		t.start();
	}

	private static ArrayList<BulletNifModel> bulletNifModels = new ArrayList<BulletNifModel>();

	private static void processDir(File dir)
	{
		System.out.println("Processing directory " + dir);
		File[] fs = dir.listFiles();
		for (int i = 0; i < fs.length; i++)
		{
			try
			{
				if (fs[i].isFile() && (fs[i].getName().endsWith(".nif") || fs[i].getName().endsWith(".kf")))
				{

				}
				else if (fs[i].isDirectory())
				{
					processDir(fs[i]);
				}

			}
			catch (Exception ex)
			{
				ex.printStackTrace();
			}
		}
	}

	private static Thread displayThread;

	private static void startDisplay()
	{
		if (displayThread == null)
		{
			displayThread = new Thread(new Runnable()
			{

				@Override
				public void run()
				{

					DebugOutput.initDebug(dynamicsWorld);
					while (true)
					{

						LWJGL.step();
						try
						{
							Thread.sleep(100);
						}
						catch (Exception e)
						{
							e.printStackTrace();
						}
					}

				}
			});
			displayThread.setDaemon(true);
			displayThread.start();
		}

	}

}