using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace ClothSim.Integration
{
    class ClothSetupWindow : EditorWindow
    {
        [MenuItem("Window/ClothSetup")]
        private static void ShowWindow()
        {
            GetWindow<ClothSetupWindow>().Init();
        }

        private int m_sizeX;
        private int m_sizeY;

        private float m_distance;

        private void Init()
        {
            Show();
        }

        private void OnGUI()
        {
            GUILayout.BeginHorizontal();
            m_sizeX = Mathf.Max(1, EditorGUILayout.IntField("Size X", m_sizeX));
            m_sizeY = Mathf.Max(1, EditorGUILayout.IntField("Size Y", m_sizeY));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            m_distance = Mathf.Max(0, EditorGUILayout.FloatField("Distance", Mathf.Max(m_distance,.01f)));
            GUILayout.EndHorizontal();

            if (GUILayout.Button("Create"))
            {
                ClothParticleObject[,] particleMapping = new ClothParticleObject[m_sizeX, m_sizeY];
                GameObject particlePhysicUpdater=new GameObject("ParticlePhysics");
                particlePhysicUpdater.AddComponent<ClothParticleSystemUpdater>();
                GameObject cloth = new GameObject("Cloth");
                cloth.transform.SetParent(particlePhysicUpdater.transform);
                for (int x = 0; x < m_sizeX; x++)
                {
                    for (int y = 0; y < m_sizeY; y++)
                    {
                        GameObject g = new GameObject(string.Format("P {0} {1}", x, y));
                        g.transform.localPosition = new Vector3(x * m_distance, y * -m_distance);
                        g.transform.SetParent(cloth.transform);

                        ClothParticleObject p = g.AddComponent<ClothParticleObject>();
                        p.Kinematic = y == 0;
                        p.Mass = 1 - (float) (y*y) / (m_sizeY*m_sizeY);

                        particleMapping[x, y] = p;
                        
                    }
                }
                //hooked the joints
                for(int x = 0; x < m_sizeX; x++)
                {
                    for (int y = 0; y < m_sizeY; y++)
                    {
                        ClothParticleObject p = particleMapping[x, y];

                        Stack<ClothParticleObject> neighbours = new Stack<ClothParticleObject>();
                        //right
                        int rightIndex = x + 1;
                        if (rightIndex < m_sizeX)
                        {
                            neighbours.Push(particleMapping[rightIndex, y]);
                        }
                        //Bottom
                        int bottomIndex = y + 1;
                        if (bottomIndex < m_sizeY)
                        {
                            neighbours.Push(particleMapping[x, bottomIndex]);
                        }
                        //diagonal right
                        if (rightIndex < m_sizeX && bottomIndex < m_sizeY)
                        {
                            neighbours.Push(particleMapping[rightIndex, bottomIndex]);
                        }
                        //diagonal left
                        int leftIndex = x - 1;
                        if (leftIndex >= 0 && bottomIndex < m_sizeY)
                        {
                            neighbours.Push(particleMapping[leftIndex, bottomIndex]);
                        }

                        ClothParticleConstraints c = p.gameObject.AddComponent<ClothParticleConstraints>();
                        c.OtherClothParticles = neighbours.ToArray();
                    }
                }
            }
        }
    }
}