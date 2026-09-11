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

        public void DrawObjects()
        {
            // Loopa igenom motorns alla objekt och rita dem på skärmen
            foreach (var body in Bodies)
            {
                int bodyPositionX = (int)body.Position.X;
                int bodyPositionY = (int)body.Position.Y;
                switch (body.objectType)
                {
                    case ObjectType.Ball:
                        Raylib.DrawCircle(bodyPositionX, bodyPositionY, body.radius, Color.DarkPurple);
                        break;
                    case ObjectType.Box:
                        
                        Raylib.DrawRectangle(bodyPositionX, bodyPositionY, (int)body.radius * 2, (int)body.radius * 2, Color.DarkPurple);
                        break;
                }


                // Rita en liten linje som visar hastighetsriktningen (Visualisering!)
                Raylib.DrawLine(
                    (int)body.Position.X,
                    (int)body.Position.Y,
                    (int)(body.Position.X + body.Velocity.X * 0.1f),
                    (int)(body.Position.Y + body.Velocity.Y * 0.1f),
                    Color.Green
                );
            }
        }

        private bool CheckCollider(PhysicsObject a, PhysicsObject b, float? radius)
        {
            if(a.objectType == ObjectType.Ball && b.objectType == ObjectType.Ball)
            {
                Vector2D difference = b.Position - a.Position;
                float distance = difference.Length();

                return distance <= a.radius + b.radius;
            }

            if(a.objectType == ObjectType.Box && b.objectType == ObjectType.Box)
            {
                float aHalf = a.radius;
                float bHalf = a.radius;

                return MathF.Abs(a.Position.X - b.Position.X) <= aHalf + bHalf && MathF.Abs(a.Position.Y - b.Position.Y) <= aHalf + bHalf;
            }

            return false;
           
        }

        private void HandleScreenBoundaries(PhysicsObject body, float width, float height, float radius)
        {
            // Golv
            if (body.Position.Y >= height - radius * 2f)
            {
                body.Position.Y = height - radius * 2f;
                body.Velocity.Y *= -body.Restitution;
            }
            // Vänster vägg
            if (body.Position.X <= radius * 2f)
            {
                body.Position.X = radius * 2f;
                body.Velocity.X *= -body.Restitution;
            }
            // Höger vägg
            if (body.Position.X >= width - radius * 2f)
            {
                body.Position.X = width - radius * 2f;
                body.Velocity.X *= -body.Restitution;
            }
        }

        private void ResolveCollision(PhysicsObject a, PhysicsObject b, float radius)
        {
            // läeger til frition
            float frition = a.Friction * b.Friction;

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

            float restitution = MathF.Min(a.Restitution, b.Restitution)/ velocityAlongNormal * -frition;
            // gör att den inte balar ur när friton är över 1
            

            float impulseMagnitude = -(1f + restitution) * velocityAlongNormal / totalInverseMass;

            Vector2D impulse = normal * impulseMagnitude;

            if (!a.IsStatic)
                a.Velocity -= impulse * a.InverseMass;

            if (!b.IsStatic)
                b.Velocity += impulse * b.InverseMass;
        }
    }
}