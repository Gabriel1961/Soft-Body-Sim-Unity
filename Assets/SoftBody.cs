using Assets;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Data.Common;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;


[RequireComponent(typeof(MeshCollider))]
[RequireComponent(typeof(MeshFilter))]
public class SoftBody : MonoBehaviour
{
    private Mesh mesh;
    private MeshCollider meshCollider;
    public float springConstant = 1;
    public float gravity = 1;
    public float friction = 20;
    public float tempreture = 1;
    public float gasConst = 1;
    public float gasQuantity = 1;
    // pv=qrt
    List<Particle> particles;
    SortedSet<ParticleSystemGenerator.Edge>[] conGraph;
    ParticleSystemGenerator.Triangle[] triangles;
    int[] revVertLkp; // gets the Particle idx from the vertex index in mesh
    // Start is called before the first frame update
    void Start()
    {
        meshCollider = GetComponent<MeshCollider>();
        mesh = GetComponent<MeshFilter>().mesh;
        (particles,conGraph,triangles,revVertLkp) = ParticleSystemGenerator.Generate(mesh);
    }

    private void OnCollisionEnter(Collision collision)
    {
        Func<float,float,float,float,float> CalcCollision = (m1,m2,v1,v2) =>
        {
            return (m1 - m2) / (m1 + m2) * v1 + 2 * m2 / (m1 + m2) * v2;
        }; // returns v1

        var p = collision.GetContact(0).point;
        var n = collision.GetContact(0).normal;
        RaycastHit hitInfo;
        Debug.DrawRay(collision.collider.transform.position, -n);
        if(Physics.Raycast(new Ray(collision.collider.transform.position, n),out hitInfo, 100,~LayerMask.NameToLayer("softbody")))
        {
            if (hitInfo.collider != meshCollider)
                return;
            int midx = hitInfo.triangleIndex*3;
            int a = revVertLkp[midx];
            int b = revVertLkp[midx+1];
            int c = revVertLkp[midx+2];

            Vector3 final = n * CalcCollision(
                particles[a].mass + particles[b].mass + particles[c].mass,
                collision.rigidbody.mass,
                (particles[a].velocity.magnitude+ particles[b].velocity.magnitude+ particles[c].velocity.magnitude)/3,
                collision.rigidbody.velocity.magnitude);
            
            particles[a].velocity = final;
            particles[b].velocity = final;
            particles[c].velocity = final;
        }
    }

    private float ComputeVolume()
    {
        Vector3 max = Vector3.negativeInfinity, min = Vector3.positiveInfinity;
        for(int i=0;i<particles.Count;i++)
        {
            max = Vector3.Max(max, particles[i].position);
            min = Vector3.Min(min, particles[i].position);
        }
        Vector3 dif = max - min;
        return dif.x * dif.y * dif.z;
    }

    private Vector3[] ComputeInternalPressureForces(float pressure)
    {
        Vector3[] forces = new Vector3[particles.Count];

        for(int i=0;i<triangles.Length;i++)
        {
            Vector3 a = particles[triangles[i].a].position;
            Vector3 b = particles[triangles[i].b].position;
            Vector3 c = particles[triangles[i].c].position;

            float area = Vector3.Cross(b-a,c-a).magnitude;
            Vector3 p = triangles[i].norm * area * pressure/3;
            forces[triangles[i].a] += p; 
            forces[triangles[i].b] += p;
            forces[triangles[i].c] += p;
        }
        return forces;
    }

    private void ComputeSpringParticle(int fst, Vector3 externalForce)
    {
        var part = particles[fst];
        
        #region Coll Dect

        // floor check
        const float floor = 0;
        float actualy = transform.InverseTransformPoint(new Vector3(0, floor, 0)).y;
        if (part.position.y < actualy)
        {
            part.position.y = actualy;
            part.velocity = -part.velocity;
        }

        #endregion
        
        var grav = gravity * Vector3.down;
        part.velocity += part.mass * (externalForce) * Time.fixedDeltaTime;
        part.velocity -= part.velocity / friction;
        part.velocity += grav * Time.fixedDeltaTime;
        part.position += part.velocity * Time.fixedDeltaTime;
    }

    private void FixedUpdate()
    {
        var springForces = ParticleSystemGenerator.ComputeSpringForces(particles,conGraph,springConstant);

        float pressure = gasConst * tempreture * gasQuantity / ComputeVolume();
        var pressForces = ComputeInternalPressureForces(pressure);

        for (int i=0;i<particles.Count;i++)
        {
            ComputeSpringParticle(i, springForces[i] + pressForces[i]);
        }
    }


    // Update is called once per frame
    void Update()
    {
        Vector3[] varr = new Vector3[mesh.vertexCount];
        for (int i = 0; i < particles.Count; i++)
        {
            foreach (int v in particles[i].refr)
            {
                varr[v] = particles[i].position;
            }
        }
        mesh.vertices = varr;
        meshCollider.sharedMesh = mesh;
    }
}
