package nifbullet.convert;

import java.util.WeakHashMap;

import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import nif.NiObjectList;
import nif.niobject.NiAVObject;
import nif.niobject.NiTriShape;
import nif.niobject.NiTriShapeData;
import nif.niobject.RootCollisionNode;
import nifbullet.util.NifBulletUtil;
import tools3d.utils.Utils3D;
import utils.convert.ConvertFromHavok;

import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CompoundShape;
import com.bulletphysics.collision.shapes.TriangleIndexVertexArray;
import com.bulletphysics.extras.gimpact.GImpactMeshShape;
import com.bulletphysics.linearmath.Transform;
import com.sun.j3d.utils.geometry.GeometryInfo;

public abstract class RootCollisionNodeToCollisionShape
{
	/**
	 * Convinience for non dynamic shapes
	 * @param bhkShape
	 * @param niToJ3dData
	 * @param scale 
	 * @return
	 */

	//NOTE ONLY scale ==1 in here!
	private static WeakHashMap<RootCollisionNode, CollisionShape> preloadedScale1Shapes = new WeakHashMap<RootCollisionNode, CollisionShape>();

	public static CollisionShape processRootCollisionNode(RootCollisionNode rootCollisionNode, NiObjectList niToJ3dData, float scale)
	{
		CollisionShape ret = null;
		if (scale == 1)
			ret = preloadedScale1Shapes.get(rootCollisionNode);
		if (ret != null)
			return ret;

		CompoundShape cs = new CompoundShape();
		for (int i = 0; i < rootCollisionNode.numChildren; i++)
		{
			NiAVObject child = (NiAVObject) niToJ3dData.get(rootCollisionNode.children[i]);
			if (child != null)
			{
				if (child instanceof NiTriShape)
				{
					NiTriShape niTriShape = (NiTriShape) child;
					NiTriShapeData data = (NiTriShapeData) niToJ3dData.get(niTriShape.data);

					CollisionShape shape = processNiTriStripsData(data, false, scale);

					Quat4f q = ConvertFromHavok.toJ3dQ4f(niTriShape.rotation);
					Vector3f v = new Vector3f(ConvertFromHavok.toJ3dP3fNif(niTriShape.translation, scale));

					Transform t = NifBulletUtil.createTrans(q, v);

					cs.addChildShape(t, shape);
				}
			}
		}
		cs.recalculateLocalAabb();
		ret = cs;

		if (ret != null && scale == 1)
			preloadedScale1Shapes.put(rootCollisionNode, ret);
		return ret;
	}

	public static CollisionShape processNiTriStripsData(NiTriShapeData data, boolean isDynamic, float scale)
	{
		GeometryInfo gi = new GeometryInfo(GeometryInfo.TRIANGLE_ARRAY);

		if (data.hasVertices)
		{
			//OPTOMIZATION
			/*
			Point3f[] vertices = new Point3f[data.numVertices];
			for (int i = 0; i < data.numVertices; i++)
			{
				vertices[i] = ConvertFromNif.toJ3dP3f(data.vertices[i], scale);
			}
			gi.setCoordinates(vertices);*/
			// scale all vertices
			float[] origVerts = Utils3D.extractArrayFromFloatBuffer(data.verticesOptBuf);
			float[] verts = new float[origVerts.length];
			for (int i = 0; i < verts.length; i++)
				verts[i] = origVerts[i] * scale;
			gi.setCoordinates(verts);

		}

		if (data.hasTriangles)
		{
			gi.setCoordinateIndices(data.trianglesOpt);
			gi.setUseCoordIndexOnly(true);
		}
		// *******************************************************************************************

		gi.compact();
		int[] coordIndices = gi.getCoordinateIndices();
		coordIndices = NifBulletUtil.remove2dTriangles(coordIndices);

		float[] coords = NifBulletUtil.makePrimitive(gi.getCoordinates());
		// float[] normals = NifBulletUtil.makePrimitive(gi.getNormals());

		TriangleIndexVertexArray indexVertexArrays = new TriangleIndexVertexArray(coordIndices.length / 3,
				NifBulletUtil.convertToIndexBuffer(coordIndices), 4 * 3, coords.length / 3, NifBulletUtil.convertToByteBuffer(coords),
				4 * 3);

		//NOTE!!!!!!!!!!!!!! there is no default CONCAVE CONCAVE collider so we MUST use GImpact or collision system throws null pointers!!
		if (isDynamic)
		{
			GImpactMeshShape trimesh = new GImpactMeshShape(indexVertexArrays);
			trimesh.updateBound();
			return trimesh;
		}
		else
		{
			BvhTriangleMeshShape trimesh = new BvhTriangleMeshShape(indexVertexArrays, true);
			return trimesh;
		}
	}

}
