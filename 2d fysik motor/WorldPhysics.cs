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
        private readonly List<PhysicsObject> _screenWalls = new List<PhysicsObject>();
        private float _wallScreenWidth = float.NaN;
        private float _wallScreenHeight = float.NaN;

        // Lägg till ett nytt objekt i motorn
        public PhysicsObject AddBody(Vector2D position, float mass,float friction, float? radius, float? width, float? height, ObjectType objectType)
        {
            // supply default rotation 0 and unwrap width/height
            PhysicsObject body = new PhysicsObject(position, mass, friction, 0f, radius, width.GetValueOrDefault(), height.GetValueOrDefault(), objectType);
            Bodies.Add(body);
            return body;
        }

        // Uppdatera alla objekt i världen
        public void Step(float deltaTime, float screenWidth, float screenHeight)
        {
            UpdateScreenWalls(screenWidth, screenHeight);

            foreach (var body in Bodies)
            {
                // 1. Lägg på gravitation
                body.AddForce(Gravity * body.Mass);

                // 2. Uppdatera position och hastighet
                body.Update(deltaTime);
            }
            for (int i = 0; i < Bodies.Count; i++)
            {
                for (int j = i + 1; j < Bodies.Count; j++)
                {
                    PhysicsObject a = Bodies[i];
                    PhysicsObject b = Bodies[j];
                    
                    if (TryGetCollision(a, b, out Vector2D normal, out float penetration, out Vector2D contactPoint))
                    {
                        ResolveCollision(a, b, normal, penetration, contactPoint);
                    }
                }
            }

            foreach (PhysicsObject body in Bodies)
            {
                foreach (PhysicsObject wall in _screenWalls)
                {
                    if (TryGetCollision(body, wall, out Vector2D normal, out float penetration, out Vector2D contactPoint))
                        ResolveCollision(body, wall, normal, penetration, contactPoint);
                }
            }
        }

        private void UpdateScreenWalls(float width, float height)
        {
            if (width == _wallScreenWidth && height == _wallScreenHeight)
                return;

            const float thickness = 20f;
            _screenWalls.Clear();
            _screenWalls.Add(new PhysicsObject(new Vector2D(-thickness / 2f, height / 2f), 0f, 1f, 0f, null, thickness, height + thickness * 2f, ObjectType.Box));
            _screenWalls.Add(new PhysicsObject(new Vector2D(width + thickness / 2f, height / 2f), 0f, 1f, 0f, null, thickness, height + thickness * 2f, ObjectType.Box));
            _screenWalls.Add(new PhysicsObject(new Vector2D(width / 2f, -thickness / 2f), 0f, 1f, 0f, null, width + thickness * 2f, thickness, ObjectType.Box));
            _screenWalls.Add(new PhysicsObject(new Vector2D(width / 2f, height + thickness / 2f), 0f, 1f, 0f, null, width + thickness * 2f, thickness, ObjectType.Box));
            _wallScreenWidth = width;
            _wallScreenHeight = height;
        }

       
       

        private void ResolveCollision(PhysicsObject a, PhysicsObject b, Vector2D normal, float penetration, Vector2D contactPoint)
        {
            float totalInverseMass = a.InverseMass + b.InverseMass;

            if (totalInverseMass <= 0)
                return;

            if (Vector2D.Dot(b.Position - a.Position, normal) < 0f)
                normal *= -1f;

            Vector2D correction = normal * (penetration / totalInverseMass);
            if (!a.IsStatic) a.Position -= correction * a.InverseMass;
            if (!b.IsStatic) b.Position += correction * b.InverseMass;

            Vector2D rA = contactPoint - a.Position;
            Vector2D rB = contactPoint - b.Position;
            Vector2D velocityA = GetVelocityAtPoint(a, rA);
            Vector2D velocityB = GetVelocityAtPoint(b, rB);
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
                : Math.Clamp(MathF.Min(a.Restitution, b.Restitution), 0f, 1f);
            float normalImpulseMagnitude = -(1f + restitution) * velocityAlongNormal / normalDenominator;
            Vector2D normalImpulse = normal * normalImpulseMagnitude;

            ApplyCollisionImpulse(a, -1f, rA, normalImpulse);
            ApplyCollisionImpulse(b, 1f, rB, normalImpulse);

            velocityA = GetVelocityAtPoint(a, rA);
            velocityB = GetVelocityAtPoint(b, rB);
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

            ApplyCollisionImpulse(a, -1f, rA, frictionImpulse);
            ApplyCollisionImpulse(b, 1f, rB, frictionImpulse);
        }

        private static float Cross(Vector2D a, Vector2D b) => a.X * b.Y - a.Y * b.X;

        private static Vector2D GetVelocityAtPoint(PhysicsObject body, Vector2D offset)
        {
            return body.Velocity + new Vector2D(-body.AngularVelocity * offset.Y, body.AngularVelocity * offset.X);
        }

        private static void ApplyCollisionImpulse(PhysicsObject body, float direction, Vector2D offset, Vector2D impulse)
        {
            if (body.IsStatic)
                return;

            body.Velocity += impulse * (direction * body.InverseMass);
            body.AngularVelocity += direction * Cross(offset, impulse) * body.InverseInertia;
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
                        DrawEllipseBody(body);
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

        private static void DrawEllipseBody(PhysicsObject body)
        {
            const int segments = 32;
            float cos = MathF.Cos(body.Rotation);
            float sin = MathF.Sin(body.Rotation);
            Vector2D previous = body.Position + new Vector2D(body.HalfWidth * cos, body.HalfWidth * sin);

            for (int i = 1; i <= segments; i++)
            {
                float angle = i * MathF.Tau / segments;
                Vector2D local = new Vector2D(body.HalfWidth * MathF.Cos(angle), body.HalfHeight * MathF.Sin(angle));
                Vector2D current = body.Position + new Vector2D(
                    local.X * cos - local.Y * sin,
                    local.X * sin + local.Y * cos);

                Raylib.DrawTriangle(
                    new Vector2(body.Position.X, body.Position.Y),
                    new Vector2(previous.X, previous.Y),
                    new Vector2(current.X, current.Y),
                    Color.DarkPurple);
                Raylib.DrawLine((int)previous.X, (int)previous.Y, (int)current.X, (int)current.Y, Color.DarkPurple);
                previous = current;
            }
        }

        private bool TryGetCollision(PhysicsObject a, PhysicsObject b, out Vector2D normal, out float penetration, out Vector2D contactPoint)
        {
            normal = Vector2D.Zero;
            penetration = 0f;
            contactPoint = Vector2D.Zero;

            Vector2D[] pointsA = a.objectType == ObjectType.Box ? GetBoxCorners(a) : a.GetEllipsePoints();
            Vector2D[] pointsB = b.objectType == ObjectType.Box ? GetBoxCorners(b) : b.GetEllipsePoints();
            return TryGetPolygonCollision(pointsA, pointsB, out normal, out penetration, out contactPoint);
        }

        private bool TryGetPolygonCollision(Vector2D[] pointsA, Vector2D[] pointsB, out Vector2D normal, out float penetration, out Vector2D contactPoint)
        {
            normal = Vector2D.Zero;
            penetration = float.MaxValue;
            contactPoint = Vector2D.Zero;

            for (int shape = 0; shape < 2; shape++)
            {
                Vector2D[] points = shape == 0 ? pointsA : pointsB;
                for (int i = 0; i < points.Length; i++)
                {
                    Vector2D edge = points[(i + 1) % points.Length] - points[i];
                    Vector2D axis = new Vector2D(-edge.Y, edge.X).Normalized();
                    if (axis.Length() == 0f)
                        continue;

                    ProjectBox(pointsA, axis, out float minA, out float maxA);
                    ProjectBox(pointsB, axis, out float minB, out float maxB);
                    float overlap = MathF.Min(maxA, maxB) - MathF.Max(minA, minB);
                    if (overlap < 0f)
                        return false;

                    if (overlap < penetration)
                    {
                        penetration = overlap;
                        normal = axis;
                    }
                }
            }

            Vector2D centerA = Vector2D.Zero;
            Vector2D centerB = Vector2D.Zero;
            foreach (Vector2D point in pointsA) centerA += point;
            foreach (Vector2D point in pointsB) centerB += point;
            centerA /= pointsA.Length;
            centerB /= pointsB.Length;

            if (Vector2D.Dot(centerB - centerA, normal) < 0f)
                normal *= -1f;

            contactPoint = (GetSupportPoint(pointsA, normal) + GetSupportPoint(pointsB, normal * -1f)) * 0.5f;
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
            float rotation = body.Rotation;

            float cos = MathF.Cos(rotation);
            float sin = MathF.Sin(rotation);

            float width = body.HalfWidth * 2f;
            float height = body.HalfHeight * 2f;

            Vector2D corner1 = new Vector2D( body.Position.X + (-width / 2f * cos - -height / 2f * sin), body.Position.Y + (-width / 2f * sin + -height / 2f * cos));

            Vector2D corner2 = new Vector2D(body.Position.X + (width / 2f * cos - -height / 2f * sin),body.Position.Y + (width / 2f * sin + -height / 2f * cos));

            Vector2D corner3 = new Vector2D(body.Position.X + (width / 2f * cos - height / 2f * sin),body.Position.Y + (width / 2f * sin + height / 2f * cos));

            Vector2D corner4 = new Vector2D(body.Position.X + (-width / 2f * cos - height / 2f * sin), body.Position.Y + (-width / 2f * sin + height / 2f * cos));

            return new Vector2D[]
            {
            corner1,
            corner2,
            corner3,
            corner4
            };
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
                    ? GetSupportPoint(body.GetEllipsePoints(), normal)
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