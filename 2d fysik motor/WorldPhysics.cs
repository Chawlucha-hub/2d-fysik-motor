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
            deltaTime = Math.Clamp(deltaTime, 0f, 0.05f);
            if (deltaTime == 0f || Bodies.Count == 0)
                return;

            float maximumSpeed = 0f;
            float smallestFeature = float.MaxValue;
            foreach (var body in Bodies)
            {
                maximumSpeed = MathF.Max(maximumSpeed, body.Velocity.Length());
                float featureSize = body.objectType == ObjectType.Ball
                    ? body.radius.GetValueOrDefault()
                    : MathF.Min(body.width.GetValueOrDefault(), body.height.GetValueOrDefault());
                smallestFeature = MathF.Min(smallestFeature, MathF.Max(featureSize, 1f));
            }

            float maximumTravel = (maximumSpeed * 2f + Gravity.Length() * deltaTime) * deltaTime;
            float requiredSubsteps = MathF.Max(
                deltaTime / (1f / 120f),
                maximumTravel / (smallestFeature * 0.5f));
            int substepCount = Math.Clamp((int)MathF.Ceiling(MathF.Min(requiredSubsteps, 128f)), 1, 128);
            float substepDeltaTime = deltaTime / substepCount;

            for (int step = 0; step < substepCount; step++)
            {
                foreach (var body in Bodies)
                {
                    body.AddForce(Gravity * body.Mass);
                    body.Update(substepDeltaTime);
                    HandleScreenBoundaries(body, screenWidth, screenHeight);
                }

                for (int iteration = 0; iteration < 4; iteration++)
                {
                    for (int i = 0; i < Bodies.Count; i++)
                    {
                        for (int j = i + 1; j < Bodies.Count; j++)
                        {
                            PhysicsObject a = Bodies[i];
                            PhysicsObject b = Bodies[j];

                            if (TryGetCollision(a, b, out Vector2D normal, out float penetration, out Vector2D contactPoint))
                                ResolveCollision(a, b, normal, penetration, contactPoint);
                        }
                    }
                }
            }
        }

        private void ResolveCollision(PhysicsObject a, PhysicsObject b, Vector2D normal, float penetration, Vector2D contactPoint)
        {
            float totalInverseMass = a.InverseMass + b.InverseMass;

            if (totalInverseMass <= 0)
                return;

            const float penetrationSlop = 0.01f;
            const float correctionPercent = 0.8f;
            float correctionDepth = MathF.Max(penetration - penetrationSlop, 0f);
            Vector2D correction = normal * (correctionDepth * correctionPercent / totalInverseMass);
            if (!a.IsStatic) a.Position -= correction * a.InverseMass;
            if (!b.IsStatic) b.Position += correction * b.InverseMass;

            Vector2D rA = contactPoint - a.Position;
            Vector2D rB = contactPoint - b.Position;
            Vector2D velocityA = a.Velocity + new Vector2D(-a.AngularVelocity * rA.Y, a.AngularVelocity * rA.X);
            Vector2D velocityB = b.Velocity + new Vector2D(-b.AngularVelocity * rB.Y, b.AngularVelocity * rB.X);
            Vector2D relativeVelocity = velocityB - velocityA;
            float velocityAlongNormal = Vector2D.Dot(relativeVelocity, normal);

            if (velocityAlongNormal >= 0f)
                return;

            float crossNormalA = Cross(rA, normal);
            float crossNormalB = Cross(rB, normal);
            float normalDenominator = totalInverseMass +
                crossNormalA * crossNormalA * a.InverseInertia +
                crossNormalB * crossNormalB * b.InverseInertia;

            if (normalDenominator <= 0f)
                return;

            float restitution = MathF.Abs(velocityAlongNormal) < 30f
                ? 0f
                : MathF.Min(a.Restitution, b.Restitution);
            float normalImpulseMagnitude = -(1f + restitution) * velocityAlongNormal / normalDenominator;
            Vector2D normalImpulse = normal * normalImpulseMagnitude;

            if (!a.IsStatic)
            {
                a.Velocity -= normalImpulse * a.InverseMass;
                a.AngularVelocity -= Cross(rA, normalImpulse) * a.InverseInertia;
            }

            if (!b.IsStatic)
            {
                b.Velocity += normalImpulse * b.InverseMass;
                b.AngularVelocity += Cross(rB, normalImpulse) * b.InverseInertia;
            }

            velocityA = a.Velocity + new Vector2D(-a.AngularVelocity * rA.Y, a.AngularVelocity * rA.X);
            velocityB = b.Velocity + new Vector2D(-b.AngularVelocity * rB.Y, b.AngularVelocity * rB.X);
            relativeVelocity = velocityB - velocityA;

            Vector2D tangent = new Vector2D(-normal.Y, normal.X);
            float crossTangentA = Cross(rA, tangent);
            float crossTangentB = Cross(rB, tangent);
            float tangentDenominator = totalInverseMass +
                crossTangentA * crossTangentA * a.InverseInertia +
                crossTangentB * crossTangentB * b.InverseInertia;

            float frictionImpulseMagnitude = tangentDenominator > 0f
                ? -Vector2D.Dot(relativeVelocity, tangent) / tangentDenominator
                : 0f;
            float frictionCoefficient = (a.Friction + b.Friction) / 2f;
            float maxFrictionImpulse = normalImpulseMagnitude * frictionCoefficient;
            frictionImpulseMagnitude = Math.Clamp(frictionImpulseMagnitude, -maxFrictionImpulse, maxFrictionImpulse);
            Vector2D frictionImpulse = tangent * frictionImpulseMagnitude;

            if (!a.IsStatic)
            {
                a.Velocity -= frictionImpulse * a.InverseMass;
                a.AngularVelocity -= Cross(rA, frictionImpulse) * a.InverseInertia;
            }

            if (!b.IsStatic)
            {
                b.Velocity += frictionImpulse * b.InverseMass;
                b.AngularVelocity += Cross(rB, frictionImpulse) * b.InverseInertia;
            }
        }

        private static float Cross(Vector2D a, Vector2D b) => a.X * b.Y - a.Y * b.X;

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

        private bool TryGetCollision(PhysicsObject a, PhysicsObject b, out Vector2D normal, out float penetration, out Vector2D contactPoint)
        {
            normal = Vector2D.Zero;
            penetration = 0f;
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
                contactPoint = a.Position + normal * (a.radius.GetValueOrDefault() - penetration * 0.5f);

                return true;
            }

            if (a.objectType == ObjectType.Box && b.objectType == ObjectType.Box)
            {
                return TryGetBoxBoxCollision(a, b, out normal, out penetration, out contactPoint);
            }

            PhysicsObject ball = a.objectType == ObjectType.Ball ? a : b;
            PhysicsObject box = a.objectType == ObjectType.Box ? a : b;

            Vector2D ballLocalPosition = Rotate(ball.Position - box.Position, -box.Rotation);
            Vector2D closestLocalPoint = new Vector2D(
                Math.Clamp(ballLocalPosition.X, -box.HalfWidth, box.HalfWidth),
                Math.Clamp(ballLocalPosition.Y, -box.HalfHeight, box.HalfHeight));
            Vector2D localBoxToBall = ballLocalPosition - closestLocalPoint;
            float distanceToBox = localBoxToBall.Length();
            float ballRadius = ball.radius.GetValueOrDefault();

            if (distanceToBox >= ballRadius)
                return false;

            if (distanceToBox > 0f)
            {
                localBoxToBall /= distanceToBox;
                penetration = ballRadius - distanceToBox;
                contactPoint = box.Position + Rotate(closestLocalPoint, box.Rotation);
            }
            else
            {
                float distanceToVerticalSide = box.HalfWidth - MathF.Abs(ballLocalPosition.X);
                float distanceToHorizontalSide = box.HalfHeight - MathF.Abs(ballLocalPosition.Y);
                float distanceToSide;

                if (distanceToVerticalSide < distanceToHorizontalSide)
                {
                    float direction = ballLocalPosition.X;
                    localBoxToBall = new Vector2D(direction == 0f ? 1f : MathF.Sign(direction), 0f);
                    penetration = ballRadius + distanceToVerticalSide;
                    distanceToSide = distanceToVerticalSide;
                }
                else
                {
                    float direction = ballLocalPosition.Y;
                    localBoxToBall = new Vector2D(0f, direction == 0f ? 1f : MathF.Sign(direction));
                    penetration = ballRadius + distanceToHorizontalSide;
                    distanceToSide = distanceToHorizontalSide;
                }

                Vector2D localContactPoint = ballLocalPosition + localBoxToBall * distanceToSide;
                contactPoint = box.Position + Rotate(localContactPoint, box.Rotation);
            }

            Vector2D boxToBall = Rotate(localBoxToBall, box.Rotation);
            normal = a.objectType == ObjectType.Ball ? boxToBall * -1f : boxToBall;
            return true;   
        }

        private static Vector2D Rotate(Vector2D vector, float angle)
        {
            float cos = MathF.Cos(angle);
            float sin = MathF.Sin(angle);
            return new Vector2D(vector.X * cos - vector.Y * sin, vector.X * sin + vector.Y * cos);
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
            float angle = body.Rotation;

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
            float angle = body.Rotation;

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
            float bestValue = Vector2D.Dot(corners[0], direction);
            for (int i = 1; i < corners.Length; i++)
            {
                float value = Vector2D.Dot(corners[i], direction);
                if (value > bestValue)
                    bestValue = value;
            }

            Vector2D supportCenter = Vector2D.Zero;
            int supportCount = 0;
            for (int i = 0; i < corners.Length; i++)
            {
                if (bestValue - Vector2D.Dot(corners[i], direction) <= 0.001f)
                {
                    supportCenter += corners[i];
                    supportCount++;
                }
            }

            return supportCenter / supportCount;
        }

        private void HandleScreenBoundaries(PhysicsObject body, float width, float height)
        {
            Vector2D[] wallNormals =
            {
                new Vector2D(-1f, 0f),
                new Vector2D(1f, 0f),
                new Vector2D(0f, -1f),
                new Vector2D(0f, 1f)
            };
            float[] wallOffsets = { 0f, width, 0f, height };

            for (int i = 0; i < wallNormals.Length; i++)
            {
                Vector2D normal = wallNormals[i];
                Vector2D supportPoint = body.objectType == ObjectType.Ball
                    ? body.Position + normal * body.BoundingRadius
                    : GetSupportPoint(GetBoxCorners(body), normal);
                float penetration = Vector2D.Dot(supportPoint, normal) - wallOffsets[i];

                if (penetration > 0f)
                    ResolveBoundaryCollision(body, normal, penetration, supportPoint);
            }
        }
        private void ResolveBoundaryCollision(PhysicsObject body, Vector2D normal, float penetration, Vector2D contactPoint)
        {
            if (body.IsStatic)
                return;

            Vector2D contactOffset = contactPoint - body.Position;
            body.Position -= normal * penetration;

            Vector2D contactVelocity = body.Velocity + new Vector2D(
                -body.AngularVelocity * contactOffset.Y,
                body.AngularVelocity * contactOffset.X);
            float velocityAlongNormal = Vector2D.Dot(contactVelocity * -1f, normal);

            if (velocityAlongNormal >= 0f)
                return;

            float angularNormal = Cross(contactOffset, normal);
            float normalDenominator = body.InverseMass +
                angularNormal * angularNormal * body.InverseInertia;

            if (normalDenominator <= 0f)
                return;

            float restitution = -velocityAlongNormal < 30f ? 0f : body.Restitution;
            float normalImpulseMagnitude = -(1f + restitution) * velocityAlongNormal / normalDenominator;
            Vector2D normalImpulse = normal * normalImpulseMagnitude;

            Vector2D tangent = new Vector2D(-normal.Y, normal.X);
            float angularTangent = Cross(contactOffset, tangent);
            float tangentDenominator = body.InverseMass +
                angularTangent * angularTangent * body.InverseInertia;
            float frictionImpulseMagnitude = tangentDenominator > 0f
                ? -Vector2D.Dot(contactVelocity * -1f, tangent) / tangentDenominator
                : 0f;
            float maximumFrictionImpulse = normalImpulseMagnitude * body.Friction;
            frictionImpulseMagnitude = Math.Clamp(
                frictionImpulseMagnitude,
                -maximumFrictionImpulse,
                maximumFrictionImpulse);

            Vector2D totalImpulse = normalImpulse + tangent * frictionImpulseMagnitude;
            body.Velocity -= totalImpulse * body.InverseMass;
            body.AngularVelocity -= Cross(contactOffset, totalImpulse) * body.InverseInertia;
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