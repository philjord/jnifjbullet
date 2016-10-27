package nifbullet.convert;

import org.jogamp.java3d.Transform3D;
import org.jogamp.vecmath.Matrix4f;
import org.jogamp.vecmath.Point3d;
import org.jogamp.vecmath.Point3f;
import org.jogamp.vecmath.Quat4f;
import org.jogamp.vecmath.Vector3d;
import org.jogamp.vecmath.Vector3f;

import nif.NifVer;
import nif.compound.NifSphereBV;
import nif.compound.NifTriangle;
import nif.compound.NifbhkCMSDChunk;
import nif.compound.NifbhkCMSDTransform;
import nif.niobject.NiTriStripsData;
import nif.niobject.bhk.bhkBoxShape;
import nif.niobject.bhk.bhkCapsuleShape;
import nif.niobject.bhk.bhkCompressedMeshShapeData;
import nif.niobject.bhk.bhkConvexVerticesShape;
import nif.niobject.bhk.bhkMultiSphereShape;
import nif.niobject.bhk.bhkSphereShape;
import nif.niobject.bhk.hkPackedNiTriStripsData;
import nifbullet.util.NifBulletUtil;
import tools3d.utils.Utils3D;
import utils.convert.ConvertFromHavok;

import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.bulletphysics.collision.shapes.CapsuleShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CompoundShape;
import com.bulletphysics.collision.shapes.ConvexHullShape;
import com.bulletphysics.collision.shapes.IndexedMesh;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.collision.shapes.TriangleIndexVertexArray;
import com.bulletphysics.dom.HeightfieldTerrainShape;
import com.bulletphysics.extras.gimpact.GImpactMeshShape;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;
import com.sun.j3d.utils.geometry.GeometryInfo;

/**
 * NOTE! compundShapes are expensive adn cause jitter possibly multi shper will be ok?
 * @author philip
 *
 */
public abstract class BhkCollisionToNifBullet
{
	public static CollisionShape bhkSphereShape(bhkSphereShape data, float scale, NifVer nifVer)
	{
		float radius = ConvertFromHavok.toJ3d(data.radius, scale, nifVer);
		SphereShape s = new SphereShape(radius);
		return s;
	}

	public static CollisionShape bhkBoxShape(bhkBoxShape data, float scale, NifVer nifVer)
	{
		Vector3f v = ConvertFromHavok.toJ3dExtents(data.dimensions, scale, nifVer);
		BoxShape b = new BoxShape(v);
		return b;

		//ConeShape appears to work better
		//SphereShape does not work
	}

	public static CollisionShape bhkMultiSphereShape(bhkMultiSphereShape data, float scale, NifVer nifVer)
	{

		CompoundShape cs = new CompoundShape();

		for (int i = 0; i < data.numSpheres; i++)
		{
			NifSphereBV sphere = data.spheres[i];

			float radius = ConvertFromHavok.toJ3d(sphere.radius, scale, nifVer);
			Vector3f loc = ConvertFromHavok.toJ3d(sphere.center, scale, nifVer);
			SphereShape ss = new SphereShape(radius);
			cs.addChildShape(NifBulletUtil.createTrans(loc), ss);
		}
		return cs;
	}

	public static CollisionShape bhkCapsuleShape(bhkCapsuleShape data, float scale, NifVer nifVer)
	{
		float radius = ConvertFromHavok.toJ3d(data.radius, scale, nifVer);
		Vector3f v1 = ConvertFromHavok.toJ3d(data.firstPoint, scale, nifVer);
		float radius1 = ConvertFromHavok.toJ3d(data.radius1, scale, nifVer);
		Vector3f v2 = ConvertFromHavok.toJ3d(data.secondPoint, scale, nifVer);
		float radius2 = ConvertFromHavok.toJ3d(data.radius2, scale, nifVer);

		if (radius != radius1 || radius != radius2)
		{
			System.out.println("bhkCapsuleShape radius != radius1 || radius != radius2 ");
		}

		float length = new Point3f(v2).distance(new Point3f(v1));

		Vector3f diff = new Vector3f(v2);

		diff.sub(v1);
		diff.scale(0.5f);
		v2.sub(diff);

		Transform3D t1 = new Transform3D();
		Transform3D t2 = new Transform3D();

		// note up must not be parallel to diff hence crazy up vector
		t1.lookAt(new Point3d(v2), new Point3d(v1), new Vector3d(diff.y, diff.z, diff.x));
		t1.invert();

		t2.rotX(Math.PI / 2);
		t1.mul(t2);

		CapsuleShape gcs = new CapsuleShape(radius, length);
		Matrix4f m = new Matrix4f();
		t1.get(m);
		Transform gt = new Transform(m);
		CompoundShape parent = new CompoundShape();
		parent.addChildShape(gt, gcs);

		return parent;
	}

	public static CollisionShape bhkConvexVerticesShape(bhkConvexVerticesShape data, float scale, NifVer nifVer)
	{
		ObjectArrayList<Vector3f> points = new ObjectArrayList<Vector3f>();

		for (int i = 0; i < data.numVertices; i++)
		{
			points.add(ConvertFromHavok.toJ3d(data.vertices[i], scale, nifVer));
		}

		ConvexHullShape chs = new ConvexHullShape(points);

		return chs;
	}

	public static float CMD_VERT_SCALE = 1f / 1000f;

	public static CollisionShape bhkCompressedMeshShape(bhkCompressedMeshShapeData data, boolean isDynamic, float scale, NifVer nifVer)
	{

		//the masks are just low 17 bits for tri and highest bit for winding
		if (data.BitsPerIndex != 17 || data.BitsPerWindingIndex != 18)
		{
			System.out.println("unexpected bhkCompressedMeshShapeData.BitsPerIndex " + data.BitsPerIndex);
			System.out.println("unexpected bhkCompressedMeshShapeData.BitesPerWindingIndex " + data.BitsPerWindingIndex);
		}

		TriangleIndexVertexArray indexVertexArrays = new TriangleIndexVertexArray();

		if (data.NumBigTris > 0)
		{
			Point3f[] vertices = new Point3f[data.BigVerts.length];
			for (int i = 0; i < data.BigVerts.length; i++)
			{
				vertices[i] = ConvertFromHavok.toJ3dP3f(//
						((data.BigVerts[i].x)), //
						((data.BigVerts[i].y)), //
						((data.BigVerts[i].z)), scale, nifVer);
			}

			int[] listPoints = new int[data.BigTris.length * 3];
			for (int i = 0; i < data.BigTris.length; i++)
			{
				listPoints[(i * 3) + 0] = data.BigTris[i].Triangle1;
				listPoints[(i * 3) + 1] = data.BigTris[i].Triangle2;
				listPoints[(i * 3) + 2] = data.BigTris[i].Triangle3;
			}

			GeometryInfo gi = new GeometryInfo(GeometryInfo.TRIANGLE_ARRAY);
			gi.setCoordinates(vertices);
			gi.setCoordinateIndices(listPoints);
			gi.setUseCoordIndexOnly(true);
			Transform t = NifBulletUtil.newIdentityTransform();

			addMesh(indexVertexArrays, gi, t);

		}

		for (int c = 0; c < data.NumChunks; c++)
		{
			NifbhkCMSDChunk chunk = data.Chunks[c];

			Point3f[] vertices = new Point3f[chunk.Vertices.length / 3];
			for (int i = 0; i < chunk.Vertices.length / 3; i++)
			{
				vertices[i] = ConvertFromHavok.toJ3dP3f(//
						((chunk.Vertices[(i * 3) + 0]) * CMD_VERT_SCALE) + chunk.translation.x, //
						((chunk.Vertices[(i * 3) + 1]) * CMD_VERT_SCALE) + chunk.translation.y, //
						((chunk.Vertices[(i * 3) + 2]) * CMD_VERT_SCALE) + chunk.translation.z, scale, nifVer);
			}

			int numStrips = chunk.NumStrips;
			int[] stripLengths = new int[numStrips];

			// copy and get full length
			int stripsLensIdxCount = 0;
			for (int i = 0; i < numStrips; i++)
			{
				stripLengths[i] = chunk.Strips[i];
				stripsLensIdxCount += chunk.Strips[i];
			}

			//NOTE one indices list hold both strip and list data
			int[] stripPoints = new int[stripsLensIdxCount];
			int idx = 0;
			for (int i = 0; i < numStrips; i++)
			{
				for (int j = 0; j < stripLengths[i]; j++)
				{
					stripPoints[idx] = chunk.Indices[idx];
					idx++;
				}
			}

			//NOTE one indices list hold both strip and list data
			int triListIndicesLength = chunk.NumIndices - stripsLensIdxCount;
			int[] listPoints = new int[triListIndicesLength];
			idx = 0;
			for (int i = stripsLensIdxCount; i < chunk.NumIndices; i++)
			{
				listPoints[idx] = chunk.Indices[i];
				idx++;
			}

			NifbhkCMSDTransform cmsdt = data.ChunkTransforms[chunk.transformIndex];
			Vector3f transformTrans = ConvertFromHavok.toJ3d(cmsdt.Translation, scale, nifVer);
			Quat4f transformRot = ConvertFromHavok.toJ3d(cmsdt.Rotation);
			Transform t = NifBulletUtil.createTrans(transformRot, transformTrans);

			// do the strips first 
			if (stripLengths.length > 0)
			{
				GeometryInfo gi = new GeometryInfo(GeometryInfo.TRIANGLE_STRIP_ARRAY);
				gi.setCoordinates(vertices);
				gi.setStripCounts(stripLengths);
				gi.setCoordinateIndices(stripPoints);
				gi.setUseCoordIndexOnly(true);

				addMesh(indexVertexArrays, gi, t);

			}

			//now the tri list
			if (triListIndicesLength > 0)
			{
				GeometryInfo gi = new GeometryInfo(GeometryInfo.TRIANGLE_ARRAY);
				gi.setCoordinates(vertices);
				gi.setCoordinateIndices(listPoints);
				gi.setUseCoordIndexOnly(true);

				addMesh(indexVertexArrays, gi, t);
			}
		}

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

	private static void addMesh(TriangleIndexVertexArray indexVertexArrays, GeometryInfo gi, Transform t)
	{
		gi.convertToIndexedTriangles();

		int[] coordIndices = gi.getCoordinateIndices();
		coordIndices = NifBulletUtil.remove2dTriangles(coordIndices);

		Point3f[] points = gi.getCoordinates();
		// transform (bake in) all coords by t	
		NifBulletUtil.transformPoints(points, t);
		float[] coords = NifBulletUtil.makePrimitive(points);

		IndexedMesh mesh = new IndexedMesh();

		mesh.numTriangles = coordIndices.length / 3;
		mesh.triangleIndexBase = NifBulletUtil.convertToIndexBuffer(coordIndices);
		mesh.triangleIndexStride = 4 * 3;
		mesh.numVertices = coords.length / 3;
		mesh.vertexBase = NifBulletUtil.convertToByteBuffer(coords);
		mesh.vertexStride = 4 * 3;

		indexVertexArrays.addIndexedMesh(mesh);

	}

	public static CollisionShape hkPackedNiTriStripsData(hkPackedNiTriStripsData data, boolean isDynamic, float scale, float xScale,
			float yScale, float zScale, NifVer nifVer)
	{

		int[] coordIndices = new int[data.numTriangles * 3];
		for (int i = 0; i < data.numTriangles; i++)
		{
			NifTriangle tri = data.triangles[i].triangle;

			coordIndices[i * 3 + 0] = tri.v1;
			coordIndices[i * 3 + 1] = tri.v2;
			coordIndices[i * 3 + 2] = tri.v3;
		}

		// copy the coords across
		float[] coords = new float[data.numVertices * 3];
		for (int i = 0; i < data.numVertices; i++)
		{
			Vector3f v = ConvertFromHavok.toJ3d(data.vertices[i], scale, nifVer);
			coords[i * 3 + 0] = v.x * xScale;
			coords[i * 3 + 1] = v.y * yScale;
			coords[i * 3 + 2] = v.z * zScale;
		}

		coordIndices = NifBulletUtil.remove2dTriangles(coordIndices);

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

	public static CollisionShape processNiTriStripsData(NiTriStripsData data, boolean isDynamic, float scale)
	{
		GeometryInfo gi = new GeometryInfo(GeometryInfo.TRIANGLE_STRIP_ARRAY);

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

		if (data.hasPoints)
		{
			int numStrips = data.numStrips;
			int[] stripLengths = data.stripLengths;

			// get full length
			int length = 0;
			for (int i = 0; i < numStrips; i++)
			{
				length += data.points[i].length;
			}

			gi.setStripCounts(stripLengths);
			int[] points = new int[length];
			int idx = 0;
			for (int i = 0; i < numStrips; i++)
			{
				for (int j = 0; j < stripLengths[i]; j++)
				{
					points[idx] = data.points[i][j];
					idx++;
				}
			}

			gi.setCoordinateIndices(points);
			gi.setUseCoordIndexOnly(true);
		}
		// *******************************************************************************************

		gi.convertToIndexedTriangles();
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

	public static CollisionShape makeFromGeometryInfo(GeometryInfo gi)
	{
		gi.convertToIndexedTriangles();

		int[] coordIndices = gi.getCoordinateIndices();
		coordIndices = NifBulletUtil.remove2dTriangles(coordIndices);

		float[] coords = NifBulletUtil.makePrimitive(gi.getCoordinates());
		// float[] normals = NifBulletUtil.makePrimitive(gi.getNormals());

		TriangleIndexVertexArray indexVertexArrays = new TriangleIndexVertexArray(coordIndices.length / 3,
				NifBulletUtil.convertToIndexBuffer(coordIndices), 4 * 3, coords.length / 3, NifBulletUtil.convertToByteBuffer(coords),
				4 * 3);

		BvhTriangleMeshShape trimesh = new BvhTriangleMeshShape(indexVertexArrays, true);
		return trimesh;

	}

	/**
	 * returns a terrain
	 * 
	 * @param heights
	 * @return
	 */

	public static HeightfieldTerrainShape makeHeightfieldTerrainShape(float[][] heights, float terrainSquareSize)
	{
		float heightScale = 1;
		int upAxis = 1;
		boolean flipQuadEdges = false;

		int heightStickWidth = heights.length;
		int heightStickLength = heights[0].length;
		float minHeight = Float.POSITIVE_INFINITY;
		float maxHeight = Float.NEGATIVE_INFINITY;
		float[] heightfieldData = new float[heightStickWidth * heightStickLength];
		for (int w = 0; w < heightStickWidth; w++)
		{
			for (int h = 0; h < heightStickLength; h++)
			{
				float x = heights[w][h];
				minHeight = minHeight > x ? x : minHeight;
				maxHeight = maxHeight < x ? x : maxHeight;

				heightfieldData[(w * heightStickLength) + h] = x;
			}
		}

		HeightfieldTerrainShape heightfieldTerrainShape = new HeightfieldTerrainShape(heightStickWidth, heightStickLength, heightfieldData,
				heightScale, minHeight, maxHeight, upAxis, flipQuadEdges);
		
		//FIXME:!!!! this has hard coding from J3dLAND in it!!!
		heightfieldTerrainShape.setLocalScaling(new Vector3f(terrainSquareSize, 1, terrainSquareSize));
		return heightfieldTerrainShape;

	}

}
