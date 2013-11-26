package nifbullet.kin;

import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

import javax.media.j3d.Behavior;
import javax.media.j3d.TransformGroup;
import javax.media.j3d.WakeupOnTransformChange;
import javax.media.j3d.WakeupOr;

import nif.j3d.J3dNiAVObject;
import nif.j3d.NiToJ3dData;
import nif.j3d.NifTransformGroup;

public class SceneGraphTransformChangeBehavior extends Behavior
{
	private WakeupOr wakeUp;

	private NBSimpleKinematicRigidBody destination;

	private List<TransformGroup> allTransformGroups = new ArrayList<TransformGroup>();

	public SceneGraphTransformChangeBehavior(J3dNiAVObject source, NBSimpleKinematicRigidBody destination, NiToJ3dData niToJ3dData)
	{
		this.destination = destination;

		J3dNiAVObject current = source;
		while (current != null)
		{
			NifTransformGroup nmtg = current.getTransformGroup();

			if (!nmtg.isNoImpact())
			{
				allTransformGroups.add(nmtg);
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

	public void initialize()
	{
		wakeupOn(wakeUp);
	}

	@SuppressWarnings(
	{ "unchecked", "rawtypes" })
	public void processStimulus(Enumeration critiria)
	{
		destination.updateInternalWorldTransform();

		//reset the wakeup
		wakeupOn(wakeUp);
	}

}
