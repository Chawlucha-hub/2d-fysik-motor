using Raylib_cs;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static System.Runtime.InteropServices.JavaScript.JSType;

namespace _2d_fysik_motor
{
    internal class WorldPhysics
    {
        public List<PhysicsObject> Bodies { get; } = new List<PhysicsObject>();
        public Vector2D Gravity { get; set; } = new Vector2D(0, 980f); // Standard-gravitation

        // Lägg till ett nytt objekt i motorn
        public PhysicsObject AddBody(Vector2D position, float mass,float friction, float? radius, float? width, float? height, ObjectType objectType)
        {
            PhysicsObject body = new PhysicsObject(position, mass, friction, radius, width, height, objectType);
            Bodies.Add(body);
            return body;
        }

        // Uppdatera alla objekt i världen
        public void Step(float deltaTime, float screenWidth, float screenHeight)
        {
            foreach (var body in Bodies)
            {
                // Choose a non-null radius: prefer explicit radius, else use the larger of width/height,
                // else fall back to 0
                float radius = body.BoundingRadius;

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
                    
                    if (TryGetCollision(a, b, out Vector2D normal, out float penetration, out Vector2D contactPoint, out Vector2D AngulerVeloscity))
                    {
                        ResolveCollision(a, b, normal, penetration, contactPoint, AngulerVeloscity);
                    }
                }
            }
        }

       
       

        private void ResolveCollision(PhysicsObject a, PhysicsObject b, Vector2D normal, float penetration, Vector2D contactPoint, Vector2D AngulerVeloscity)
        {

            float friktonkofisient = (a.Friction + b.Friction) / 2;

            float totalInverseMass = a.InverseMass + b.InverseMass;

            if (totalInverseMass <= 0)
                return;

            Vector2D correction = normal * (penetration / totalInverseMass);

            Vector2D rA = contactPoint - a.Position;
            Vector2D rB = contactPoint - b.Position;

            Vector2D VelocityA = a.Velocity + new Vector2D(-a.AngulerVeloscity.Y * rA.Y, a.AngulerVeloscity.X * rA.X);
            Vector2D VelocityB = b.Velocity + new Vector2D(-b.AngulerVeloscity.Y * rB.Y, b.AngulerVeloscity.X * rB.X);
            Vector2D relativeVelocity = VelocityB - VelocityA;

            float velocityAlongNormal = Vector2D.Dot(relativeVelocity, normal);

            if (velocityAlongNormal > 0)
                return;

            float restitution = MathF.Min(a.Restitution, b.Restitution);

            float impulseMagnitude = -(1f + restitution) * velocityAlongNormal / totalInverseMass;

            Vector2D impulse = normal * impulseMagnitude;

            if (!a.IsStatic)
                a.Velocity -= impulse * a.InverseMass;

            if (!b.IsStatic)
                b.Velocity += impulse * b.InverseMass;

            Vector2D relativinpulsPosColition = b.Velocity - a.Velocity;

            Vector2D tangent = new Vector2D(-normal.Y, normal.X);

            float velosetyalongtanhent = Vector2D.Dot(relativinpulsPosColition, tangent);

            float frictioInpulsmangnetud = -velosetyalongtanhent / totalInverseMass;

            float maxfricion = impulseMagnitude * friktonkofisient;

            if (frictioInpulsmangnetud > maxfricion)
            {
                frictioInpulsmangnetud = maxfricion;
            }
            else if (frictioInpulsmangnetud < -maxfricion)
            {
                frictioInpulsmangnetud = -maxfricion;
            }

            Vector2D friktionInpuls = tangent * frictioInpulsmangnetud;

            

            if (!a.IsStatic)
                a.Velocity -= friktionInpuls * a.InverseMass;

            if (!b.IsStatic)
                b.Velocity += friktionInpuls * b.InverseMass;

        }

        public void DrawObjects()
        {
            foreach (var body in Bodies)
            {
                int bodyPositionX = (int)body.Position.X;
                int bodyPositionY = (int)body.Position.Y;
                switch (body.objectType)
                {
                    case ObjectType.Ball:
                        int circleRadius = (int)(body.radius ?? 0f);
                        Raylib.DrawCircle(bodyPositionX, bodyPositionY, circleRadius, Color.DarkPurple);
                        break;
                    case ObjectType.Box:
                        Raylib.DrawRectangle(
                            (int)(body.Position.X - body.HalfWidth),
                            (int)(body.Position.Y - body.HalfHeight),
                            (int)(body.HalfWidth * 2f),
                            (int)(body.HalfHeight * 2f),
                            Color.DarkPurple);
                        break;
                }

                Raylib.DrawLine(
                    (int)body.Position.X,
                    (int)body.Position.Y,
                    (int)(body.Position.X + body.Velocity.X * 0.1f),
                    (int)(body.Position.Y + body.Velocity.Y * 0.1f),
                    Color.Green
                );
            }
        }

        private bool TryGetCollision(PhysicsObject a, PhysicsObject b, out Vector2D normal, out float penetration,out Vector2D contactPoint, out Vector2D AngulerVeloscity)
        {
            normal = Vector2D.Zero;
            penetration = 0f;
            contactPoint = Vector2D.Zero;
            AngulerVeloscity = Vector2D.Zero;

            if (a.objectType == ObjectType.Ball && b.objectType == ObjectType.Ball)
            {
                Vector2D difference = b.Position - a.Position;
                float distance = difference.Length();
                float combinedRadius = a.radius.GetValueOrDefault() + b.radius.GetValueOrDefault();

                if (distance >= combinedRadius)
                    return false;

                normal = distance == 0f ? new Vector2D(1f, 0f) : difference / distance;
                penetration = combinedRadius - distance;
                return true;
            }

            if (a.objectType == ObjectType.Box && b.objectType == ObjectType.Box)
            {
                float overlapX = a.HalfWidth + b.HalfWidth - MathF.Abs(a.Position.X - b.Position.X);
                float overlapY = a.HalfHeight + b.HalfHeight - MathF.Abs(a.Position.Y - b.Position.Y);

                if (overlapX <= 0f || overlapY <= 0f)
                    return false;

                if (overlapX < overlapY)
                {
                    float direction = b.Position.X - a.Position.X;
                    normal = new Vector2D(direction == 0f ? 1f : MathF.Sign(direction), 0f);
                    penetration = overlapX;
                }
                else
                {
                    float direction = b.Position.Y - a.Position.Y;
                    normal = new Vector2D(0f, direction == 0f ? 1f : MathF.Sign(direction));
                    penetration = overlapY;
                }

                return true;
            }

            PhysicsObject ball = a.objectType == ObjectType.Ball ? a : b;
            PhysicsObject box = a.objectType == ObjectType.Box ? a : b;

            Vector2D closestPoint = new Vector2D(
                Math.Clamp(ball.Position.X, box.Position.X - box.HalfWidth, box.Position.X + box.HalfWidth),
                Math.Clamp(ball.Position.Y, box.Position.Y - box.HalfHeight, box.Position.Y + box.HalfHeight));
            Vector2D boxToBall = ball.Position - closestPoint;
            float distanceToBox = boxToBall.Length();
            float ballRadius = ball.radius.GetValueOrDefault();

            if (distanceToBox >= ballRadius)
                return false;

            if (distanceToBox > 0f)
            {
                boxToBall /= distanceToBox;
                penetration = ballRadius - distanceToBox;
            }
            else
            {
                float distanceToVerticalSide = box.HalfWidth - MathF.Abs(ball.Position.X - box.Position.X);
                float distanceToHorizontalSide = box.HalfHeight - MathF.Abs(ball.Position.Y - box.Position.Y);

                if (distanceToVerticalSide < distanceToHorizontalSide)
                {
                    float direction = ball.Position.X - box.Position.X;
                    boxToBall = new Vector2D(direction == 0f ? 1f : MathF.Sign(direction), 0f);
                    penetration = ballRadius + distanceToVerticalSide;
                }
                else
                {
                    float direction = ball.Position.Y - box.Position.Y;
                    boxToBall = new Vector2D(0f, direction == 0f ? 1f : MathF.Sign(direction));
                    penetration = ballRadius + distanceToHorizontalSide;
                }
            }

            normal = a.objectType == ObjectType.Ball ? boxToBall * -1f : boxToBall;
            return true;
        }

        private void HandleScreenBoundaries(PhysicsObject body, float width, float height, float radius)
        {
            float halfWidth = body.objectType == ObjectType.Ball ? body.BoundingRadius : body.HalfWidth;
            float halfHeight = body.objectType == ObjectType.Ball ? body.BoundingRadius : body.HalfHeight;

            // Golv
            if (body.Position.Y >= height - halfHeight)
            {
                body.Position.Y = height - halfHeight;
                body.Velocity.Y *= -body.Restitution;
            }
            // Vänster vägg
            if (body.Position.X <= halfWidth)
            {
                body.Position.X = halfWidth;
                body.Velocity.X *= -body.Restitution;
            }
            // Höger vägg
            if (body.Position.X >= width - halfWidth)
            {
                body.Position.X = width - halfWidth;
                body.Velocity.X *= -body.Restitution;
            }
        }
        public Vector2D CalculateContactPointUniversal(PhysicsObject a, PhysicsObject b, Vector2D normal)
        {
            // Hitta punkten på A:s kant i normalens riktning
            Vector2D pointA = a.Position + normal * a.BoundingRadius;

            // Hitta punkten på B:s kant i motsatt riktning (-normal)
            Vector2D pointB = b.Position - normal * b.BoundingRadius;

            // Ta medelvärdet (mitten) av dessa två punkter
            return (pointA + pointB) * 0.5f;
        }

    }
}