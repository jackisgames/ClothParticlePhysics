using ClothSim.Physics;
using UnityEngine;

namespace ClothSim.Integration
{
    public static class ClothPhysicExtension
    {
        public static ParticleData ToParticleData(this ClothParticleObject obj)
        {
            Vector3 pos = obj.transform.position;
            ParticleData p = new ParticleData(pos.x, pos.y, pos.z, obj.Mass,obj.Kinematic);
            return p;
        }

        public static Vector3 GetPosition(this ClothParticleSystem physicSystem,int index)
        {
            float x, y, z;
            physicSystem.GetPosition(index,out x, out y, out z);

            return new Vector3(x, y, z);
        }
        public static void SetPosition(this ClothParticleSystem physicSystem, int index,Vector3 position)
        {
            physicSystem.SetPosition(index, position.x, position.y, position.z);
        }
    }
}
