using UnityEngine;

namespace ClothSim.Integration
{
    public class ClothParticleObject:MonoBehaviour
    {
        public bool Kinematic;
        public float Mass;

        public int Index { get; set; }

        private void OnDrawGizmos()
        {
            Gizmos.DrawWireSphere(transform.position,.05f);
        }
    }
}
