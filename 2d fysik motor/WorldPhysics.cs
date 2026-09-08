using Raylib_cs;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace _2d_fysik_motor
{
    internal class WorldPhysics
    {
        public List<PhysicsObject> Bodies { get; } = new List<PhysicsObject>();
        public Vector2D Gravity { get; set; } = new Vector2D(0, 980f); // Standard-gravitation

        // Lägg till ett nytt objekt i motorn
        public PhysicsObject AddBody(Vector2D position, float mass,float friction, float radius, ObjectType objectType)
        {
            PhysicsObject body = new PhysicsObject(position, mass, friction, radius, objectType);
            Bodies.Add(body);
            return body;
        }

        // Uppdatera alla objekt i världen
        public void Step(float deltaTime, float screenWidth, float screenHeight)
        {
            foreach (var body in Bodies)
            {
                float radius = body.radius;
                // 1. Lägg på gravitation
                body.AddForce(Gravity * body.Mass);

                // 2. Uppdatera position och hastighet
                body.Update(deltaTime);

                // 3. Enkel vägg- och golvkollision för skärmkanterna
                HandleScreenBoundaries(body, screenWidth, screenHeight, radius);

              
            }
            for (int i = 0; i < Bodies.Count; i++)
            {
                for (int j = i + 1; j < Bodies.Count; j++)
                {
                    PhysicsObject a = Bodies[i];
                    PhysicsObject b = Bodies[j];
                    if (CheckCollider(a, b, a.radius))
                    {
                        ResolveCollision(a, b, a.radius);
                    }
                }
            }
        }

        private bool CheckCollider(PhysicsObject a, PhysicsObject b, float radius)
        {
            Vector2D difference = b.Position - a.Position;

            float distance = difference.Length();

            return distance <= radius * 2f;
        }


        private void HandleScreenBoundaries(PhysicsObject body, float width, float height, float radius)
        {
            // Golv
            if (body.Position.Y >= height - radius)
            {
                body.Position.Y = height - radius;
                body.Velocity.Y *= -body.Restitution;
            }
            // Vänster vägg
            if (body.Position.X <= radius)
            {
                body.Position.X = radius;
                body.Velocity.X *= -body.Restitution;
            }
            // Höger vägg
            if (body.Position.X >= width - radius)
            {
                body.Position.X = width - radius;
                body.Velocity.X *= -body.Restitution;
            }
        }
        private void ResolveCollision(PhysicsObject a, PhysicsObject b, float radius)
        {
            float frition = a.Friction + b.Friction;
            Vector2D normal = b.Position - a.Position;
            float distance = normal.Length();
            if (distance == 0)
            {
                normal = new Vector2D(1, 0);
                distance = 0.001f;
            }
            else
            {
                normal = normal / distance;
            }

            float penetration = radius * 2f - distance;

            float totalInverseMass = a.InverseMass + b.InverseMass;

            if(totalInverseMass > 0 )
            {
                Vector2D correction = normal * (penetration / totalInverseMass);

                if(!a.IsStatic)
                {
                    a.Position -= correction * a.InverseMass;
                }

                if (!b.IsStatic)
                {
                    b.Position += correction * b.InverseMass;
                }
            }

            Vector2D relativeVelocity = b.Velocity - a.Velocity;

            float velocityAlongNormal = Vector2D.Dot(relativeVelocity, normal);

            if (velocityAlongNormal > 0)
                return;

            float restitution = MathF.Min(a.Restitution, b.Restitution)/frition;

            float impulseMagnitude = -(1f + restitution) * velocityAlongNormal / totalInverseMass;

            Vector2D impulse = normal * impulseMagnitude;

            if (!a.IsStatic)
                a.Velocity -= impulse * a.InverseMass;

            if (!b.IsStatic)
                b.Velocity += impulse * b.InverseMass;
        }
    }

}
