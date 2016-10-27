package nifbullet.simple;

import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

import org.jogamp.java3d.Behavior;
import org.jogamp.java3d.TransformGroup;
import org.jogamp.java3d.WakeupOnTransformChange;
import org.jogamp.java3d.WakeupOr;

import nif.j3d.J3dNiAVObject;
import nif.j3d.NiToJ3dData;

public class SceneGraphTransformChangeBehavior extends Behavior
{
	private WakeupOr wakeUp;

	private NBKinematicRigidBody destination;

	private List<TransformGroup> allTransformGroups = new ArrayList<TransformGroup>();

	public SceneGraphTransformChangeBehavior(J3dNiAVObject source, NBKinematicRigidBody destination, NiToJ3dData niToJ3dData)
	{
		this.destination = destination;

		J3dNiAVObject current = source;
		while (current != null)
		{
			if (!current.isNoImpact())
			{
				allTransformGroups.add(current);
			}
			current = niToJ3dData.get(current.getNiAVObject().parent);
		}

		ArrayList<WakeupOnTransformChange> criteria = new ArrayList<WakeupOnTransformChange>();

		for (TransformGroup tg : allTransformGroups)
		{
			tg.setCapability(TransformGroup.ALLOW_TRANSFORM_READ);
			criteria.add(new WakeupOnTransformChange(tg));
		}
		wakeUp = new WakeupOr(criteria.toArray(new WakeupOnTransformChange[criteria.size()]));
	}

	public SceneGraphTransformChangeBehavior(TransformGroup tg, NBKinematicRigidBody destination, NiToJ3dData niToJ3dData)
	{
		this.destination = destination;

		allTransformGroups.add(tg);

		ArrayList<WakeupOnTransformChange> criteria = new ArrayList<WakeupOnTransformChange>();

		tg.setCapability(TransformGroup.ALLOW_TRANSFORM_READ);
		criteria.add(new WakeupOnTransformChange(tg));

		wakeUp = new WakeupOr(criteria.toArray(new WakeupOnTransformChange[criteria.size()]));
	}

	public void initialize()
	{
		wakeupOn(wakeUp);
	}

	@SuppressWarnings({ "rawtypes" })
	public void processStimulus(Enumeration critiria)
	{
		destination.updateInternalWorldTransform();

		//reset the wakeup
		wakeupOn(wakeUp);
	}

}
