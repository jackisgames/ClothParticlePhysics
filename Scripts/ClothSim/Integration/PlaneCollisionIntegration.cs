using ClothSim.Physics;
using UnityEngine;

namespace ClothSim.Integration
{
    class PlaneCollisionIntegration:MonoBehaviour,ICollisionIntegrator
    {
        private readonly PlaneCollider m_collider = new PlaneCollider();

        private void Update()
        {
            Vector3 normal = transform.up;
            Vector3 position = transform.position;
            m_collider.SetNormal(normal.x, normal.y, normal.z);
            m_collider.SetPosition(position.x, position.y, position.z);
        }
        public ICollisionObject CollisionObject()
        {
            return m_collider;
        }

        private void OnDrawGizmos()
        {
            Gizmos.DrawLine(transform.position,transform.position+transform.up);
        }
    }
}
