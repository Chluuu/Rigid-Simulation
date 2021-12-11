using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	Vector3[] vertices;

	bool launched 		= false;
	float dt 			= 0.015f;

	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	Vector3 g           = new Vector3(0, -9.98f, 0); // Acceleration of gravity

	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision

	float velTdecay=0.5f;

	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	Matrix4x4 add(Matrix4x4 a, Matrix4x4 b){
        Matrix4x4 A = Matrix4x4.zero;
        A[0, 0] = a[0, 0] + b[0, 0];
        A[0, 1] = a[0, 1] + b[0, 1];
        A[0, 2] = a[0, 2] + b[0, 2];
        A[0, 3] = a[0, 3] + b[0, 3];
        A[1, 0] = a[1, 0] + b[1, 0];
        A[1, 1] = a[1, 1] + b[1, 1];
        A[1, 2] = a[1, 2] + b[1, 2];
        A[1, 3] = a[1, 3] + b[1, 3];
        A[2, 0] = a[2, 0] + b[2, 0];
        A[2, 1] = a[2, 1] + b[2, 1];
        A[2, 2] = a[2, 2] + b[2, 2];
        A[2, 3] = a[2, 3] + b[2, 3];
        A[3, 0] = a[3, 0] + b[3, 0];
        A[3, 1] = a[3, 1] + b[3, 1];
        A[3, 2] = a[3, 2] + b[3, 2];
        A[3, 3] = a[3, 3] + b[3, 3];
        return A;
	}

	Matrix4x4 sub(Matrix4x4 a, Matrix4x4 b){
        Matrix4x4 A = Matrix4x4.zero;
        A[0, 0] = a[0, 0] - b[0, 0];
        A[0, 1] = a[0, 1] - b[0, 1];
        A[0, 2] = a[0, 2] - b[0, 2];
        A[0, 3] = a[0, 3] - b[0, 3];
        A[1, 0] = a[1, 0] - b[1, 0];
        A[1, 1] = a[1, 1] - b[1, 1];
        A[1, 2] = a[1, 2] - b[1, 2];
        A[1, 3] = a[1, 3] - b[1, 3];
        A[2, 0] = a[2, 0] - b[2, 0];
        A[2, 1] = a[2, 1] - b[2, 1];
        A[2, 2] = a[2, 2] - b[2, 2];
        A[2, 3] = a[2, 3] - b[2, 3];
        A[3, 0] = a[3, 0] - b[3, 0];
        A[3, 1] = a[3, 1] - b[3, 1];
        A[3, 2] = a[3, 2] - b[3, 2];
        A[3, 3] = a[3, 3] - b[3, 3];
        return A;	
	}
	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Matrix4x4 R=Matrix4x4.Rotate(transform.rotation);
		Vector3 x=transform.position;

		int colCount=0;
		Vector3 moveSum=new Vector3(0,0,0);
		Vector3 velSum=new Vector3(0,0,0);
		for(int i=0;i<vertices.Length;i++)
		{
			Vector3 move=R * vertices[i];
			Vector3 pos=x+move;
			if(Vector3.Dot(pos-P,N)<0){
				Vector3 wxRi = Get_Cross_Matrix(w) * move;//Vector3.Cross(w,move);
				Vector3 vi = v + wxRi;
                if (Vector3.Dot(vi, N) < 0)
                {
					moveSum+=move;
					velSum+=vi;
					colCount++;
                }
			}
		}

		if(colCount>0){
			//collision response
			Vector3 avgMove=moveSum/colCount;
			Vector3 avgVel=velSum/colCount;

			Vector3 velN=Vector3.Dot(avgVel,N)*N;
			Vector3 velT=avgVel-velN;
            

			float a=Mathf.Max(1-velTdecay*(1+restitution)*velN.magnitude/velT.magnitude,0);
			velN = -restitution * velN;
			//velN=-restitution*velN;
			velT=a*velT;
			Vector3 velNew=velN + velT;

			Matrix4x4 matrixMove=Get_Cross_Matrix(avgMove);

			Matrix4x4 massInverse = Matrix4x4.identity;
			massInverse[0,0]=1/mass;
			massInverse[1,1]=1/mass;
			massInverse[2,2]=1/mass;
			massInverse[3,3]=1/mass;

			Matrix4x4 invI_ref=I_ref.inverse;
			invI_ref.SetRow(3,new Vector4(0,0,0,1));
			invI_ref.SetColumn(3,new Vector4(0,0,0,1));
				
			Matrix4x4 K=sub(massInverse,matrixMove*invI_ref*matrixMove);
			Vector3 J=K.inverse.MultiplyVector(velNew-avgVel);

			//update velocity
			v=v+1/mass*J;
			Vector3 dw=invI_ref*matrixMove*J;
			w=w+dw;

			restitution*=0.5f;
			
		}
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			//w = new Vector3(0.5f, 0, 0);
			launched=true;
		}

		// Part I: Update velocities
		if(launched){

			v = linear_decay * (v + dt * g);
			w = angular_decay * w;

			// Part II: Collision Impulse
			Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
			Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

			// Part III: Update position & orientation
			//Update linear status
			Vector3 x=transform.position;
			x=x+dt*v;
			
			//Update angular status
			Quaternion q = transform.rotation;
			Quaternion angular=new Quaternion(w.x*dt/2,w.y*dt/2,w.z*dt/2,0);
			angular=angular*q;
			q=new Quaternion(angular.x+q.x,angular.y+q.y,angular.z+q.z,angular.w+q.w);

			// Part IV: Assign to the object
			transform.position = x;
			transform.rotation = q;
		}
	}
}
