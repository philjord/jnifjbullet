package nifbullet.util;

import java.io.File;
import java.util.prefs.Preferences;

import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.vecmath.Vector3f;

import nif.NifToJ3d;
import nifbullet.BulletNifModel;
import nifbullet.BulletNifModelClassifier;
import utils.source.MeshSource;
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

public class NifBulletLoader
{

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

	public static void main(String[] args)
	{

		prefs = Preferences.userNodeForPackage(NifBulletLoader.class);
		String baseDir = prefs.get("NifBulletLoader.baseDir", System.getProperty("user.dir"));

		JFileChooser fc = new JFileChooser(baseDir);
		fc.setFileSelectionMode(JFileChooser.FILES_AND_DIRECTORIES);

		fc.setMultiSelectionEnabled(true);
		fc.showOpenDialog(new JFrame());

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

		if (fc.getSelectedFiles() != null)
		{

			File[] fs = fc.getSelectedFiles();
			for (File f : fs)
			{
				System.out.println("Selected file: " + f);
				prefs.put("NifBulletLoader.baseDir", f.getPath());
				if (f.isDirectory())
				{
					processDir(f);
				}
				else if (f.isFile())
				{
					try
					{
						BulletNifModelClassifier.testNif(f.getAbsolutePath(), new FileMeshSource());
						BulletNifModelClassifier.createNifBullet(f.getAbsolutePath(), new FileMeshSource(), 0).addToDynamicsWorld(
								dynamicsWorld);

					}
					catch (Exception ex)
					{
						ex.printStackTrace();
					}
				}
			}
			System.out.println("done");
		}
		System.exit(0);
	}

	private static void processDir(File dir)
	{
		MeshSource meshSource = new FileMeshSource();
		System.out.println("Processing directory " + dir);
		Thread.currentThread().setPriority(Thread.MIN_PRIORITY);
		File[] fs = dir.listFiles();
		for (int i = 0; i < fs.length; i++)
		{
			try
			{
				if (fs[i].isFile() && (fs[i].getName().endsWith(".nif") || fs[i].getName().endsWith(".kf")))
				{
					System.out.println("\tFile: " + fs[i]);
					BulletNifModelClassifier.testNif(fs[i].getAbsolutePath(), meshSource);
					BulletNifModel bnm = BulletNifModelClassifier.createNifBullet(fs[i].getAbsolutePath(), meshSource, 0);
					if (bnm != null)
						bnm.addToDynamicsWorld(dynamicsWorld);

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

		// avoid memory caching
		NifToJ3d.clearCache();
	}
}