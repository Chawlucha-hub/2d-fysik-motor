using Raylib_cs;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using System;
using static System.Runtime.InteropServices.JavaScript.JSType;

namespace _2d_fysik_motor
{
    // Hanterar fysikvärldens kroppar, simulering, kollisioner och ritning.
    internal class WorldPhysics
    {
        // Alla kroppar som ingår i simuleringen.
        public List<PhysicsObject> Bodies { get; } = new List<PhysicsObject>();

        // Gravitationsacceleration i världens koordinater.
        // Standard-gravitation
        public Vector2D Gravity { get; set; } = new Vector2D(0, 980f);

        // Statiska väggar som håller kropparna inom fönstrets gränser.
        private readonly List<PhysicsObject> _screenWalls =
            new List<PhysicsObject>();

        // Sparar fönstrets senaste storlek så väggarna bara byggs om vid behov.
        private float _wallScreenWidth = float.NaN;
        private float _wallScreenHeight = float.NaN;

        private HashSet<PhysicsObject> gravityRotatedThisStep = new();

        // Skapar en kropp med nollrotation,
        // lägger till den i världen och returnerar den.
        public PhysicsObject AddBody(
                Vector2D position, float mass, float friction, float? radius,
                float? width, float? height, ObjectType objectType)
        {
            PhysicsObject body = new PhysicsObject(
                    position, mass, friction, 0f, radius,
                    width.GetValueOrDefault(),
                    height.GetValueOrDefault(),
                    objectType
                    );
            Bodies.Add(body);
            return body;
        }

        // Stegar fysiken framåt och hanterar kroppskollisioner för en bildruta.
        public void Step(float deltaTime, float screenWidth, float screenHeight)
        {
            // Säkerställer att de statiska väggarna matchar fönstrets storlek.
            UpdateScreenWalls(screenWidth, screenHeight);

            gravityRotatedThisStep.Clear();

            // Lägg på gravitation och integrera varje kropps rörelse.
            foreach (var body in Bodies)
            {
                // Omvandlar gravitationsaccelerationen till kraft
                // med F = m * a.
                body.AddForce(Gravity * body.Mass);

                body.Update(deltaTime);
            }

            // Testar varje par av kroppar en gång
            // och löser eventuella kollisioner.
            for (int i = 0; i < Bodies.Count; i++)
            {
                for (int j = i + 1; j < Bodies.Count; j++)
                {
                    PhysicsObject a = Bodies[i];
                    PhysicsObject b = Bodies[j];

                    if (TryGetCollision(
                                a, b, out Vector2D normal,
                                out float penetration,
                                out Vector2D contactPoint))
                    {
                        ResolveCollision(a, b, normal, penetration,
                                contactPoint, deltaTime
                                );
                    }
                }
            }

            // Testar kropparna mot de statiska väggarna runt fönstret.
            foreach (PhysicsObject body in Bodies)
            {
                foreach (PhysicsObject wall in _screenWalls)
                {
                    if (TryGetCollision(
                                body, wall, out Vector2D normal,
                                out float penetration,
                                out Vector2D contactPoint))
                    {
                        ResolveCollision(
                                body, wall, normal, penetration, contactPoint,
                                deltaTime
                                );
                    }
                }
            }
        }

        // Bygger om de statiska fönsterväggarna när fönstrets storlek ändras.
        private void UpdateScreenWalls(float width, float height)
        {
            if (width == _wallScreenWidth && height == _wallScreenHeight)
                return;

            const float thickness = 20f;
            _screenWalls.Clear();
            _screenWalls.Add(new PhysicsObject(new Vector2D
                        (-thickness / 2f, height / 2f),
                        0f, 1f, 0f, null, thickness,
                        height + thickness * 2f, ObjectType.Box
                        ));
            _screenWalls.Add(new PhysicsObject(
                        new Vector2D(width + thickness / 2f, height / 2f),
                        0f, 1f, 0f, null, thickness,
                        height + thickness * 2f, ObjectType.Box
                        ));
            _screenWalls.Add(new PhysicsObject(
                        new Vector2D(width / 2f, -thickness / 2f),
                        0f, 1f, 0f, null,
                        width + thickness * 2f, thickness, ObjectType.Box
                        ));
            _screenWalls.Add(new PhysicsObject(
                        new Vector2D(width / 2f, height + thickness / 2f),
                        0f, 1f, 0f, null,
                        width + thickness * 2f, thickness, ObjectType.Box
                        ));
            _wallScreenWidth = width;
            _wallScreenHeight = height;
        }

        // Korrigerar överlappning och beräknar kollisionsimpulser
        // med studs och friktion.
        private void ResolveCollision(PhysicsObject a, PhysicsObject b,
                Vector2D normal, float penetration, Vector2D contactPoint,
                float deltaTime)
        {
            float totalInverseMass = a.InverseMass + b.InverseMass;

            if (totalInverseMass <= 0)
                return;

            if (Vector2D.Dot(b.Position - a.Position, normal) < 0f)
                normal *= -1f;
            // Kontaktpunktens hastighet inkluderar både rörelse och rotation.
            Vector2D rA = contactPoint - a.Position;
            Vector2D rB = contactPoint - b.Position;
            Vector2D velocityA = GetVelocityAtPoint(a, rA);
            Vector2D velocityB = GetVelocityAtPoint(b, rB);
            Vector2D relativeVelocity = velocityB - velocityA;
            float velocityAlongNormal = Vector2D.Dot(relativeVelocity, normal);

            // Fördelar positionskorrigeringen enligt kropparnas inversa massor.
            Vector2D correction = normal * (penetration / totalInverseMass);
            if (!a.IsStatic) a.Position -= correction * a.InverseMass;
            if (!b.IsStatic) b.Position += correction * b.InverseMass;

            if (a.objectType != b.objectType)
            {
                PhysicsObject ellipse =
                    a.objectType == ObjectType.Ball ? a : b;

                PhysicsObject box =
                    a.objectType == ObjectType.Box ? a : b;

                if (!gravityRotatedThisStep.Contains(ellipse))
                {
                    ApplyGravityRotation(ellipse, box, deltaTime);

                    gravityRotatedThisStep.Add(ellipse);
                }
            }

            if (velocityAlongNormal >= 0f)
                return;

            float crossNormalA = Cross(rA, normal);
            float crossNormalB = Cross(rB, normal);
            float normalDenominator = totalInverseMass +
                crossNormalA * crossNormalA * a.InverseInertia +
                crossNormalB * crossNormalB * b.InverseInertia;

            if (normalDenominator <= 0f)
                return;

            // Dämpar studs vid låga hastigheter
            // för att undvika små, oönskade studs.
            float restitution = Math.Clamp(
                    MathF.Min(a.Restitution, b.Restitution), 0f, 1f
                    );
            float normalImpulseMagnitude =
                -(1f + restitution) * velocityAlongNormal / normalDenominator;
            Vector2D normalImpulse = normal * normalImpulseMagnitude;

            ApplyCollisionImpulse(a, -1f, rA, normalImpulse);
            ApplyCollisionImpulse(b, 1f, rB, normalImpulse);

            velocityA = GetVelocityAtPoint(a, rA);
            velocityB = GetVelocityAtPoint(b, rB);
            relativeVelocity = velocityB - velocityA;

            // Friktionen verkar längs tangenten
            // och begränsas av normalimpulsens storlek.
            Vector2D tangent = new Vector2D(-normal.Y, normal.X);
            float crossTangentA = Cross(rA, tangent);
            float crossTangentB = Cross(rB, tangent);
            float tangentDenominator = totalInverseMass +
                crossTangentA * crossTangentA * a.InverseInertia +
                crossTangentB * crossTangentB * b.InverseInertia;

            float frictionImpulseMagnitude = tangentDenominator > 0f
                ? -Vector2D.Dot(relativeVelocity, tangent) / tangentDenominator
                : 0f;
            float frictionCoefficient = MathF.Sqrt(a.Friction * b.Friction);
            float maxFrictionImpulse =
                normalImpulseMagnitude * frictionCoefficient;
            frictionImpulseMagnitude = Math.Clamp(
                    frictionImpulseMagnitude,
                    -maxFrictionImpulse,
                    maxFrictionImpulse
                    );

            Vector2D frictionImpulse = tangent * frictionImpulseMagnitude;

            ApplyCollisionImpulse(a, -1f, rA, frictionImpulse);
            ApplyCollisionImpulse(b, 1f, rB, frictionImpulse);
        }

        // Beräknar 2D-kryssprodukten som ett skalärt värde.
        private static float Cross(Vector2D a, Vector2D b) =>
            a.X * b.Y - a.Y * b.X;

        // Beräknar kroppens hastighet vid en punkt med hänsyn till rotation.
        private static Vector2D GetVelocityAtPoint(PhysicsObject body,
                Vector2D offset)
        {
            return body.Velocity + new Vector2D(
                    -body.AngularVelocity * offset.Y,
                    body.AngularVelocity * offset.X
                    );
        }

        // Applicerar en impuls på både kroppens linjära och roterande rörelse.
        private static void ApplyCollisionImpulse(PhysicsObject body,
                float direction, Vector2D offset, Vector2D impulse)
        {
            if (body.IsStatic)
                return;

            body.Velocity += impulse * (direction * body.InverseMass);
            body.AngularVelocity +=
                direction * Cross(offset, impulse) * body.InverseInertia;
        }

        // Ritar alla kroppar och visar deras hastighetsvektorer.
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

        // Approximerar en roterad ellips med trianglar och kantlinjer.
        private static void DrawEllipseBody(PhysicsObject body)
        {
            const int segments = 128;

            float a = body.HalfWidth;
            float b = body.HalfHeight;

            float cos = MathF.Cos(body.Rotation);
            float sin = MathF.Sin(body.Rotation);

            // Börja vid den högra ändpunkten.
            Vector2D previous = new Vector2D(
                    body.Position.X + a * cos,
                    body.Position.Y + a * sin
                    );

            for (int i = 1; i <= segments; i++)
            {
                // Vinkel från 0 till PI ger en halv ellips.
                float angle = MathF.PI * 2f * i / segments;

                // Punkter på en halv ellips i lokala koordinater.
                float localX = a * MathF.Cos(angle);
                float localY = b * MathF.Sin(angle);

                // Rotera punkten och flytta den till världens position.
                Vector2D current = new Vector2D(
                        body.Position.X + localX * cos - localY * sin,
                        body.Position.Y + localX * sin + localY * cos
                        );

                // Rita bågens kant.
                Raylib.DrawLine(
                        (int)previous.X,
                        (int)previous.Y,
                        (int)current.X,
                        (int)current.Y,
                        Color.DarkPurple
                        );

                previous = current;
            }

            // Rita den raka undersidan.
            Vector2D point1 = new Vector2D(
                    body.Position.X - a * cos,
                    body.Position.Y - a * sin
                    );

            Vector2D point2 = new Vector2D(
                    body.Position.X + a * cos,
                    body.Position.Y + a * sin
                    );

            Raylib.DrawLine(
                    (int)point1.X,
                    (int)point1.Y,
                    (int)point2.X,
                    (int)point2.Y,
                    Color.DarkPurple
                    );
        }

        // Skapar punkter för respektive form
        // och testar om kropparna överlappar.
        private bool TryGetCollision(
                PhysicsObject a,
                PhysicsObject b,
                out Vector2D normal,
                out float penetration,
                out Vector2D contactPoint)
        {
            normal = Vector2D.Zero;
            penetration = 0f;
            contactPoint = Vector2D.Zero;

            Vector2D[] pointsA = a.objectType == ObjectType.Ball
                ? GetCirclePoints(a)
                : GetBoxCorners(a);

            Vector2D[] pointsB = b.objectType == ObjectType.Ball
                ? GetCirclePoints(b)
                : GetBoxCorners(b);

            return TryGetPolygonCollision(
                    pointsA, pointsB,
                    pointsA, pointsB,
                    out normal,
                    out penetration,
                    out contactPoint,
                    a, b);
        }

        // Använder separerande axlar för att hitta överlappning,
        // minsta penetration och normal.
        private bool TryGetPolygonCollision(
                Vector2D[] pointsA,
                Vector2D[] pointsB,
                Vector2D[] CircelpintsA,
                Vector2D[] CircelpintsB,
                out Vector2D normal,
                out float penetration,
                out Vector2D contactPoint,
                PhysicsObject a,
                PhysicsObject b)
        {
            normal = Vector2D.Zero;
            penetration = 0f;
            contactPoint = Vector2D.Zero;

            // ---------------------------------
            // CIRKEL MOT CIRKEL
            // ---------------------------------
            if (a.objectType == ObjectType.Ball &&
                    b.objectType == ObjectType.Ball)
            {
                Vector2D direction = b.Position - a.Position;

                float distance = direction.Length();

                if (distance < 0.0001f)
                    direction = new Vector2D(1f, 0f);
                else
                    direction /= distance;

                float radiusA = GetEllipseSupportRadius(a, direction);
                float radiusB = GetEllipseSupportRadius(b, direction * -1f);

                float overlap = radiusA + radiusB - distance;

                if (overlap <= 0f)
                    return false;

                normal = direction;
                penetration = overlap;

                contactPoint = a.Position + direction * radiusA;

                return true;
            }

            // ---------------------------------
            // CIRKEL MOT BOX
            // ---------------------------------
            if (a.objectType != b.objectType)
            {
                PhysicsObject circle =
                    a.objectType == ObjectType.Ball ? a : b;

                PhysicsObject box =
                    a.objectType == ObjectType.Box ? a : b;

                // Ellipsens centrum är dess Position.
                Vector2D ellipseCenter = circle.Position;

                float cos = MathF.Cos(box.Rotation);
                float sin = MathF.Sin(box.Rotation);

                // Ellipsens centrum i boxens lokala koordinater.
                Vector2D relative = ellipseCenter - box.Position;

                float localX = relative.X * cos + relative.Y * sin;
                float localY = -relative.X * sin + relative.Y * cos;

                // Närmaste punkt på boxen.
                float closestX = Math.Clamp(
                        localX, -box.HalfWidth, box.HalfWidth);

                float closestY = Math.Clamp(
                        localY, -box.HalfHeight, box.HalfHeight);

                // Omvandla punkten till världens koordinater.
                Vector2D closestPoint = box.Position + new Vector2D(
                        closestX * cos - closestY * sin,
                        closestX * sin + closestY * cos);

                Vector2D difference = closestPoint - ellipseCenter;
                float distance = difference.Length();

                Vector2D ellipseToBox;

                if (distance > 0.0001f)
                {
                    Vector2D direction = difference / distance;

                    float ellipseExtent =
                        GetEllipseSupportRadius(circle, direction);

                    if (distance >= ellipseExtent)
                        return false;

                    ellipseToBox = direction;
                    penetration = ellipseExtent - distance;
                }
                else
                {
                    // Ellipsens centrum ligger inuti boxen.
                    float distanceX = box.HalfWidth - MathF.Abs(localX);
                    float distanceY = box.HalfHeight - MathF.Abs(localY);

                    Vector2D outward;

                    if (distanceX < distanceY)
                    {
                        float sign = localX >= 0f ? 1f : -1f;

                        outward = new Vector2D(
                                sign * cos,
                                sign * sin);

                        closestPoint = box.Position + new Vector2D(
                                sign * box.HalfWidth * cos - localY * sin,
                                sign * box.HalfWidth * sin + localY * cos);

                        penetration =
                            distanceX +
                            GetEllipseSupportRadius(circle, outward);
                    }
                    else
                    {
                        float sign = localY >= 0f ? 1f : -1f;

                        outward = new Vector2D(
                                -sign * sin,
                                sign * cos);

                        closestPoint = box.Position + new Vector2D(
                                localX * cos - sign * box.HalfHeight * sin,
                                localX * sin + sign * box.HalfHeight * cos);

                        penetration =
                            distanceY +
                            GetEllipseSupportRadius(circle, outward);
                    }

                    ellipseToBox = outward * -1f;
                }

                // Normalen ska alltid peka från A mot B.
                normal = a.objectType == ObjectType.Ball
                    ? ellipseToBox
                    : ellipseToBox * -1f;

                // Kontaktpunkt = ellipsens verkliga ytterpunkt mot boxen.
                contactPoint = GetEllipseSupportPoint(circle, ellipseToBox);

                return true;
            }

            // ---------------------------------
            // BOX MOT BOX (DIN BEFINTLIGA SAT)
            // ---------------------------------
            penetration = float.MaxValue;

            for (int shape = 0; shape < 2; shape++)
            {
                Vector2D[] points = shape == 0 ? pointsA : pointsB;

                for (int i = 0; i < points.Length; i++)
                {
                    Vector2D edge =
                        points[(i + 1) % points.Length] - points[i];

                    Vector2D axis =
                        new Vector2D(-edge.Y, edge.X).Normalized();

                    if (axis.Length() == 0f)
                        continue;

                    ProjectBox(pointsA, axis,
                            out float minA, out float maxA);

                    ProjectBox(pointsB, axis,
                            out float minB, out float maxB);

                    float overlap =
                        MathF.Min(maxA, maxB) - MathF.Max(minA, minB);

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

            foreach (Vector2D point in pointsA)
                centerA += point;

            foreach (Vector2D point in pointsB)
                centerB += point;

            centerA /= pointsA.Length;
            centerB /= pointsB.Length;

            if (Vector2D.Dot(centerB - centerA, normal) < 0f)
                normal *= -1f;

            contactPoint = GetContactPoint(pointsA, pointsB, normal);

            return true;
        }

        // Väljer ett ungefärligt kontaktläge mellan kropparnas stödytor.
        private Vector2D GetContactPoint(Vector2D[] pointsA,
                Vector2D[] pointsB, Vector2D normal)
        {
            Vector2D tangent = new Vector2D(-normal.Y, normal.X);
            GetSupportFeature(
                    pointsA, normal, tangent, out float minTangentA,
                    out float maxTangentA, out float normalA);

            GetSupportFeature(
                    pointsB, normal * -1f, tangent, out float minTangentB,
                    out float maxTangentB, out float normalB);

            float overlapMin = MathF.Max(minTangentA, minTangentB);
            float overlapMax = MathF.Min(maxTangentA, maxTangentB);
            float tangentPosition = overlapMin <= overlapMax
                ? (overlapMin + overlapMax) * 0.5f
                : (minTangentA + maxTangentA + minTangentB + maxTangentB) *
                0.25f;

            return tangent * tangentPosition + normal *
                ((normalA + normalB) * 0.5f);
        }

        private float GetEllipseRadiusInDirection(
                PhysicsObject body,
                Vector2D direction)
        {
            float cos = MathF.Cos(body.Rotation);
            float sin = MathF.Sin(body.Rotation);

            // Riktningen omvandlas till ellipsens lokala koordinater.
            float localX = direction.X * cos + direction.Y * sin;
            float localY = -direction.X * sin + direction.Y * cos;

            float a = body.HalfWidth;
            float b = body.HalfHeight;

            float denominator = MathF.Sqrt(
                    (localX * localX) / (a * a) +
                    (localY * localY) / (b * b));

            if (denominator < 0.0001f)
                return 0f;

            return 1f / denominator;
        }

        // Hämtar stödytans ytterpunkter längs tangenten
        // och dess läge längs normalen.
        private void GetSupportFeature(
                Vector2D[] points,
                Vector2D supportDirection,
                Vector2D tangent,
                out float minTangent,
                out float maxTangent,
                out float normalPosition)
        {
            Vector2D normal = new Vector2D(tangent.Y, -tangent.X);
            float bestValue = Vector2D.Dot(points[0], supportDirection);
            for (int i = 1; i < points.Length; i++)
                bestValue = MathF.Max(
                        bestValue, Vector2D.Dot(points[i], supportDirection)
                        );

            minTangent = float.MaxValue;
            maxTangent = float.MinValue;
            normalPosition = 0f;
            int supportCount = 0;

            for (int i = 0; i < points.Length; i++){
                if (
                        bestValue - Vector2D.Dot(points[i], supportDirection)
                        > 0.001f){
                    continue;
                }

                float tangentProjection = Vector2D.Dot(points[i], tangent);
                minTangent = MathF.Min(minTangent, tangentProjection);
                maxTangent = MathF.Max(maxTangent, tangentProjection);
                normalPosition += Vector2D.Dot(points[i], normal);
                supportCount++;
            }

            normalPosition /= supportCount;
        }

        // Projicerar polygonens punkter på en axel
        // och hittar min- och maxvärdet.
        private void ProjectBox(Vector2D[] corners, Vector2D axis, out float
                min, out float max)
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
        private Vector2D[] GetCirclePoints(PhysicsObject body)
        {
            float rotation = body.Rotation;
            float radius = body.HalfWidth;

            float cos = MathF.Cos(rotation);
            float sin = MathF.Sin(rotation);

            Vector2D point1 = new Vector2D(
                    body.Position.X - radius * cos,
                    body.Position.Y - radius * sin
                    );

            Vector2D point2 = new Vector2D(
                    body.Position.X + radius * cos,
                    body.Position.Y + radius * sin
                    );

            return new Vector2D[] { point1, point2 };
        }
        private float GetEllipseSupportRadius(
                PhysicsObject body,
                Vector2D direction)
        {
            float cos = MathF.Cos(body.Rotation);
            float sin = MathF.Sin(body.Rotation);

            // Riktningen i ellipsens lokala koordinater.
            float localX = direction.X * cos + direction.Y * sin;
            float localY = -direction.X * sin + direction.Y * cos;

            float a = body.HalfWidth;
            float b = body.HalfHeight;

            return MathF.Sqrt(
                    a * a * localX * localX +
                    b * b * localY * localY
                    );
        }

        // Ellipsens ytterpunkt i en given riktning (världskoordinater).
        private Vector2D GetEllipseSupportPoint(
                PhysicsObject body, Vector2D direction){
            float cos = MathF.Cos(body.Rotation);
            float sin = MathF.Sin(body.Rotation);

            float localX = direction.X * cos + direction.Y * sin;
            float localY = -direction.X * sin + direction.Y * cos;

            float a = body.HalfWidth;
            float b = body.HalfHeight;

            float h = MathF.Sqrt(
                    a * a * localX * localX + b * b * localY * localY
                    );

            if (h < 0.0001f)
                return body.Position;

            float sx = a * a * localX / h;
            float sy = b * b * localY / h;

            return body.Position + new Vector2D(
                    sx * cos - sy * sin,
                    sx * sin + sy * cos);
        }

        private void ApplyGravityRotation(
                PhysicsObject ellipse,
                PhysicsObject box,
                float deltaTime)
        {
            float gravityLength = Gravity.Length();

            if (gravityLength < 0.0001f)
                return;

            Vector2D down = Gravity / gravityLength;

            // Kolla att boxens ovansida faktiskt pekar uppåt
            // (ellipsen vilar på en "golv-yta").
            Vector2D[] corners = GetBoxCorners(box);

            float bestDot = float.MinValue;

            for (int i = 0; i < 4; i++)
            {
                Vector2D edge = corners[(i + 1) % 4] - corners[i];
                Vector2D outward = new Vector2D(edge.Y, -edge.X).Normalized();

                float dot = Vector2D.Dot(outward, down * -1f);

                if (dot > bestDot)
                    bestDot = dot;
            }

            if (bestDot < 0.5f)
                return;

            // Ellipsens kontaktpunkt = dess stödpunkt i
            // gravitationens riktning.
            float cos = MathF.Cos(ellipse.Rotation);
            float sin = MathF.Sin(ellipse.Rotation);

            float localX = down.X * cos + down.Y * sin;
            float localY = -down.X * sin + down.Y * cos;

            float a = ellipse.HalfWidth;
            float b = ellipse.HalfHeight;

            float denominator = MathF.Sqrt(
                    a * a * localX * localX +
                    b * b * localY * localY);

            if (denominator < 0.0001f)
                return;

            float supportX = a * a * localX / denominator;
            float supportY = b * b * localY / denominator;

            Vector2D support = new Vector2D(
                    supportX * cos - supportY * sin,
                    supportX * sin + supportY * cos);

            Vector2D pivot = ellipse.Position + support;

            // Avstånd från kontaktpunkten till tyngdpunkten.
            Vector2D r = ellipse.Position - pivot;

            // Gravitationens moment runt kontaktpunkten
            // (kan vara positivt ELLER negativt).
            Vector2D gravityForce = Gravity * ellipse.Mass;
            float torque = Cross(r, gravityForce);

            float pivotInertia =
                ellipse.Inertia +
                ellipse.Mass * r.Length() * r.Length();

            if (pivotInertia < 0.0001f)
                return;

            float angularAcceleration = torque / pivotInertia;

            // Dämpning så att den gungar till ro.
            const float damping = 1.5f;
            angularAcceleration -= ellipse.AngularVelocity * damping;

            float deltaOmega = angularAcceleration * deltaTime;

            ellipse.AngularVelocity += deltaOmega;

            // Håll kontaktpunktens hastighet oförändrad.
            ellipse.Velocity += new Vector2D(
                    -deltaOmega * r.Y,
                    deltaOmega * r.X);
        }

        // Beräknar den roterade boxens fyra hörn i världens koordinater.
        private Vector2D[] GetBoxCorners(PhysicsObject body)
        {
            float rotation = body.Rotation;

            float cos = MathF.Cos(rotation);
            float sin = MathF.Sin(rotation);

            float width = body.HalfWidth * 2f;
            float height = body.HalfHeight * 2f;

            Vector2D corner1 = new Vector2D(
                    body.Position.X +
                    (-width / 2f * cos - -height / 2f * sin),
                    body.Position.Y +
                    (-width / 2f * sin + -height / 2f * cos)
                    );

            Vector2D corner2 = new Vector2D(
                    body.Position.X +
                    (width / 2f * cos - -height / 2f * sin),
                    body.Position.Y +
                    (width / 2f * sin + -height / 2f * cos)
                    );

            Vector2D corner3 = new Vector2D(
                    body.Position.X +
                    (width / 2f * cos - height / 2f * sin),
                    body.Position.Y +
                    (width / 2f * sin + height / 2f * cos)
                    );

            Vector2D corner4 = new Vector2D(
                    body.Position.X +
                    (-width / 2f * cos - height / 2f * sin),
                    body.Position.Y +
                    (-width / 2f * sin + height / 2f * cos)
                    );

            return new Vector2D[]
            {
                corner1,
                corner2,
                corner3,
                corner4
            };
        }
    }
}
