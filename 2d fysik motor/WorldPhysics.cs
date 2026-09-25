using Raylib_cs;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
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

            float friktonkofisient = (a.Friction + b.Friction) / 2f;
            

            float totalInverseMass = a.InverseMass + b.InverseMass;

            if (totalInverseMass <= 0)
                return;

            Vector2D correction = normal * (penetration / totalInverseMass);
            if (!a.IsStatic) a.Position -= correction * a.InverseMass;
            if (!b.IsStatic) b.Position += correction * b.InverseMass;

            Vector2D rA = contactPoint - a.Position;
            Vector2D rB = contactPoint - b.Position;

            Vector2D VelocityA = a.Velocity + new Vector2D(-a.AngulerVeloscity.Y * rA.Y, a.AngulerVeloscity.X * rA.X);
            Vector2D VelocityB = b.Velocity + new Vector2D(-b.AngulerVeloscity.Y * rB.Y, b.AngulerVeloscity.X * rB.X);
            Vector2D relativeVelocity = VelocityB - VelocityA;


            float rnA = rA.X * normal.Y - rA.Y * normal.X;
            float rnB = rB.X * normal.Y - rB.Y * normal.X;
           
            float totalInverseMassNormal = a.InverseMass + b.InverseMass +
                               (rnA * rnA * a.InverseInertia) +
                               (rnB * rnB * b.InverseInertia);

            float velocityAlongNormal = Vector2D.Dot(relativeVelocity, normal);

            if (velocityAlongNormal > -0.5f)
                return;

            float restitution = MathF.Min(a.Restitution, b.Restitution);

            float impulseMagnitude = -(1f + restitution) * velocityAlongNormal / totalInverseMassNormal;

            Vector2D impulse = normal * impulseMagnitude;

            Vector2D relativinpulsPosColition = VelocityB - VelocityA;

            Vector2D tangent = new Vector2D(-normal.Y, normal.X);

            float velosetyalongtanhent = Vector2D.Dot(relativinpulsPosColition, tangent);

            float rtA = rA.X * tangent.Y - rA.Y * tangent.X;
            float rtB = rB.X * tangent.Y - rB.Y * tangent.X;
            float totalInverseMassTangent = a.InverseMass + b.InverseMass + (rtA * rtA * a.InverseInertia) + (rtB * rtB * b.InverseInertia);

            float frictioInpulsmangnetud = -velosetyalongtanhent / totalInverseMassTangent;

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

            a.AngularVelocity = Math.Clamp(a.AngularVelocity, -10f, 10f);
            b.AngularVelocity = Math.Clamp(b.AngularVelocity, -10f, 10f);

            if (!a.IsStatic)
            {
                a.Velocity -= (impulse + friktionInpuls) * a.InverseMass;
                if (MathF.Abs(normal.Y) > 0.9f)
                {
                    a.AngularVelocity = 0f;
                    b.AngularVelocity = 0f;
                }

                a.AngularVelocity -= (rA.X * impulse.Y - rA.Y * impulse.X) * a.InverseInertia;
                a.AngularVelocity -= (rA.X * friktionInpuls.Y - rA.Y * friktionInpuls.X) * a.InverseInertia;
            }

            if (!b.IsStatic)
            {
                b.Velocity += (impulse + friktionInpuls) * b.InverseMass;
                if (MathF.Abs(normal.Y) > 0.9f)
                {
                    a.AngularVelocity = 0f;
                    b.AngularVelocity = 0f;
                }

                b.AngularVelocity += (rB.X * impulse.Y - rB.Y * impulse.X) * b.InverseInertia;
                b.AngularVelocity += (rB.X * friktionInpuls.Y - rB.Y * friktionInpuls.X) * b.InverseInertia;
            }
                

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

                        Rectangle rectangle = new Rectangle(
                            body.Position.X,
                            body.Position.Y,
                            body.HalfWidth * 2f,
                            body.HalfHeight * 2f
                        );

                        Vector2 origin = new Vector2(
                            body.HalfWidth,
                            body.HalfHeight
                        );

                        Raylib.DrawRectanglePro(
                            rectangle,
                            origin,
                            body.Rotation * (180f / MathF.PI),
                            Color.DarkPurple
                        );

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
            AngulerVeloscity = Vector2D.Zero;
            contactPoint = Vector2D.Zero;
            

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
                return TryGetBoxBoxCollision(a, b, out normal, out penetration, out contactPoint);
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
            contactPoint = CalculateContactPointUniversal(a, b, normal);
            return true;   
        }


        private bool TryGetBoxBoxCollision( PhysicsObject a, PhysicsObject b, out Vector2D normal, out float penetration, out Vector2D contactPoint)
        {
            normal = Vector2D.Zero;
            penetration = float.MaxValue;
            contactPoint = Vector2D.Zero;

            Vector2D[] cornersA = GetBoxCorners(a);
            Vector2D[] cornersB = GetBoxCorners(b);

            Vector2D[] axes =
            {
                GetBoxAxis(a, 0),
                GetBoxAxis(a, 1),
                GetBoxAxis(b, 0),
                GetBoxAxis(b, 1)
            };

            foreach (Vector2D axis in axes)
            {
                ProjectBox(cornersA, axis, out float minA, out float maxA);
                ProjectBox(cornersB, axis, out float minB, out float maxB);

                float overlap =
                    MathF.Min(maxA, maxB) -
                    MathF.Max(minA, minB);

                if (overlap <= 0f)
                    return false;

                if (overlap < penetration)
                {
                    penetration = overlap;

                    normal = axis;

                    // Se till att normalen pekar från A mot B
                    Vector2D centerDifference = b.Position - a.Position;

                    if (Vector2D.Dot(normal, centerDifference) < 0f)
                    {
                        normal *= -1f;
                    }
                }
            }

            // Hitta ungefärlig riktig kontaktpunkt
            Vector2D pointA = GetSupportPoint(cornersA, normal);

            Vector2D oppositeNormal = new Vector2D( -normal.X, -normal.Y);

            Vector2D pointB = GetSupportPoint(cornersB, oppositeNormal);

            contactPoint = (pointA + pointB) * 0.5f;

            return true;
        }

        private void ProjectBox(Vector2D[] corners, Vector2D axis, out float min, out float max)
        {
            min = Vector2D.Dot(corners[0], axis);
            max = min;

            for (int i = 1; i < corners.Length; i++)
            {
                float projection =
                    Vector2D.Dot(corners[i], axis);

                if (projection < min)
                    min = projection;

                if (projection > max)
                    max = projection;
            }
        }

        private Vector2D GetBoxAxis(PhysicsObject body, int index)
        {
            float angle =
                body.Rotation * (MathF.PI / 180f);

            float cos = MathF.Cos(angle);
            float sin = MathF.Sin(angle);

            if (index == 0)
            {
                return new Vector2D(cos, sin);
            }
            else
            {
                return new Vector2D(-sin, cos);
            }
        }

        private Vector2D[] GetBoxCorners(PhysicsObject body)
        {
            float angle =
                body.Rotation * (MathF.PI / 180f);

            float cos = MathF.Cos(angle);
            float sin = MathF.Sin(angle);

            Vector2D[] localCorners =
            {
                new Vector2D(-body.HalfWidth, -body.HalfHeight),
                new Vector2D( body.HalfWidth, -body.HalfHeight),
                new Vector2D( body.HalfWidth,  body.HalfHeight),
                new Vector2D(-body.HalfWidth,  body.HalfHeight)
            };

            Vector2D[] worldCorners = new Vector2D[4];

            for (int i = 0; i < 4; i++)
            {
                float x = localCorners[i].X;
                float y = localCorners[i].Y;

                worldCorners[i] =
                    new Vector2D(
                        body.Position.X + x * cos - y * sin,
                        body.Position.Y + x * sin + y * cos
                    );
            }

            return worldCorners;
        }

        private Vector2D GetSupportPoint(
    Vector2D[] corners,
    Vector2D direction)
        {
            Vector2D bestPoint = corners[0];

            float bestValue =
                Vector2D.Dot(bestPoint, direction);

            for (int i = 1; i < corners.Length; i++)
            {
                float value =
                    Vector2D.Dot(corners[i], direction);

                if (value > bestValue)
                {
                    bestValue = value;
                    bestPoint = corners[i];
                }
            }

            return bestPoint;
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
            float distanceA;

            if (MathF.Abs(normal.X) > MathF.Abs(normal.Y))
                distanceA = a.HalfWidth;
            else
                distanceA = a.HalfHeight;

            float distanceB;

            if (MathF.Abs(normal.X) > MathF.Abs(normal.Y))
                distanceB = b.HalfWidth;
            else
                distanceB = b.HalfHeight;

            Vector2D pointA = a.Position + normal * distanceA;
            Vector2D pointB = b.Position - normal * distanceB;

            return (pointA + pointB) * 0.5f;
        }

    }
}