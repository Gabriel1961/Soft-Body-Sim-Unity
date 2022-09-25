

using Unity.VisualScripting;
using UnityEngine;

namespace Assets
{
    public class Particle
    {
        public float mass;
        public Vector3 velocity;
        public Vector3 position;
        public int[] refr;


        public Vector3 CalculateSpringForce(Particle p,float k,float x0)
        {
            Vector3 dir = p.position - position;
            float len = dir.magnitude;
            Vector3 norm = dir / len;
            float force = k * (len - x0);
            return force * norm;
        }
    }

}
