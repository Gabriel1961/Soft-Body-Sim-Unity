using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Unity.VisualScripting;
using UnityEditor.Build;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.UIElements.Experimental;
using UnityEngine.VFX;

namespace Assets
{
    public static class ParticleSystemGenerator
    {
        public struct Edge : IComparable<Edge>
        {
            public int to;  // node to wich it is connected
            public float w; // weight

            public int CompareTo(Edge other)
            {
                return to.CompareTo(other.to);
            }
        }
        /// <summary>
        // !!! Winding is not preserved
        /// </summary>
        public struct Triangle
        {
            public int a,b,c;
            public Vector3 norm;
            public Triangle(int x,int y, int z, Vector3 normal)
            {
                norm = normal;
                int[] a = { x,y,z};
                Array.Sort(a);
                this.a = a[0];
                this.b = a[1];
                this.c = a[2];
            }
        }

        public static Tuple<List<Particle>, SortedSet<Edge>[], Triangle[], int[]> Generate(Mesh mesh)
        {
            var particles = new List<Particle>();
            int[] revVertLkp = new int[mesh.vertexCount];
            var verts = new Dictionary<Vector3, List<int>>();
            var triangles = new HashSet<Triangle>();
            
            for(int i=0;i<mesh.vertices.Length;i++)
            { 
                var c = mesh.vertices[i];
                List<int> val;
                if(verts.TryGetValue(c,out val))
                {
                    val.Add(i);
                }
                else 
                    verts.Add(c, new List<int>() { i });
            }

            particles = verts.Select((v) => new Particle()
            {
                mass = 1,
                position = v.Key,
                refr = v.Value.ToArray(),
                velocity = Vector3.zero
            }).ToList();

            var conGraph = new SortedSet<Edge>[particles.Count];
            
            for (int i = 0; i < particles.Count; i++)
            {
                conGraph[i] = new SortedSet<Edge>();
                for (int j = 0; j < particles[i].refr.Length; j++)
                {
                    revVertLkp[particles[i].refr[j]] = i;
                } 
            }

            for (int i = 0; i < mesh.triangles.Length; i += 3)
            {
                var tris = mesh.triangles;

                int i1 = revVertLkp[tris[i]];
                int i2 = revVertLkp[tris[i + 1]];
                int i3 = revVertLkp[tris[i + 2]];

                float d12 = (particles[i1].position - particles[i2].position).magnitude;
                float d23 = (particles[i2].position - particles[i3].position).magnitude;
                float d31 = (particles[i3].position - particles[i1].position).magnitude;
                conGraph[i1].Add(new Edge { to = i2, w = d12 });
                conGraph[i1].Add(new Edge { to = i3, w = d31 });

                conGraph[i2].Add(new Edge { to = i1, w = d12 });
                conGraph[i2].Add(new Edge { to = i3, w = d23 });
                
                conGraph[i3].Add(new Edge { to = i1, w = d31 });
                conGraph[i3].Add(new Edge { to = i2, w = d23 });

                triangles.Add(new Triangle(i1, i2, i3, mesh.normals[tris[i]]));
            }


            return new (particles,conGraph,triangles.ToArray(),revVertLkp);
        }

        public static Vector3[] ComputeSpringForces(List<Particle> particles, SortedSet<Edge>[] conGraph,float k)
        {
            var forces = new Vector3[particles.Count];
            for(int i = 0; i < conGraph.Length; i++)
            {
                foreach(var edg in conGraph[i])
                {
                    forces[i] += particles[i].CalculateSpringForce(particles[edg.to], k, edg.w);
                }
            }
            return forces;
        }
    }
}
