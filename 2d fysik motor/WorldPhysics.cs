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
    // Hanterar fysikvärldens kroppar, simulering, kollisioner och ritning.
    internal class WorldPhysics
    {
        // Alla kroppar som ingår i simuleringen.
        public List<PhysicsObject> Bodies { get; } = new List<PhysicsObject>();

        // Gravitationsacceleration i världens koordinater.
        public Vector2D Gravity { get; set; } = new Vector2D(0, 980f); // Standard-gravitation

        // Statiska väggar som håller kropparna inom fönstrets gränser.
        private readonly List<PhysicsObject> _screenWalls = new List<PhysicsObject>();

        // Sparar fönstrets senaste storlek så väggarna bara byggs om vid behov.
        private float _wallScreenWidth = float.NaN;
        private float _wallScreenHeight = float.NaN;

        // Skapar en kropp med nollrotation, lägger till den i världen och returnerar den.
        public PhysicsObject AddBody(Vector2D position, float mass, float friction, float? radius, float? width, float? height, ObjectType objectType)
        {
            PhysicsObject body = new PhysicsObject(position, mass, friction, 0f, radius, width.GetValueOrDefault(), height.GetValueOrDefault(), objectType);
            Bodies.Add(body);
            return body;
        }

        // Stegar fysiken framåt och hanterar kroppskollisioner för en bildruta.
        public void Step(float deltaTime, float screenWidth, float screenHeight)
        {
            // Säkerställer att de statiska väggarna matchar fönstrets storlek.
            UpdateScreenWalls(screenWidth, screenHeight);

            // Lägg på gravitation och integrera varje kropps rörelse.
            foreach (var body in Bodies)
            {
                // Omvandlar gravitationsaccelerationen till kraft med F = m * a.
                body.AddForce(Gravity * body.Mass);

                body.Update(deltaTime);
            }

            // Testar varje par av kroppar en gång och löser eventuella kollisioner.
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

            // Testar kropparna mot de statiska väggarna runt fönstret.
            foreach (PhysicsObject body in Bodies)
            {
                foreach (PhysicsObject wall in _screenWalls)
                {
                    if (TryGetCollision(body, wall, out Vector2D normal, out float penetration, out Vector2D contactPoint))
                        ResolveCollision(body, wall, normal, penetration, contactPoint);
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
            _screenWalls.Add(new PhysicsObject(new Vector2D(-thickness / 2f, height / 2f), 0f, 1f, 0f, null, thickness, height + thickness * 2f, ObjectType.Box));
            _screenWalls.Add(new PhysicsObject(new Vector2D(width + thickness / 2f, height / 2f), 0f, 1f, 0f, null, thickness, height + thickness * 2f, ObjectType.Box));
            _screenWalls.Add(new PhysicsObject(new Vector2D(width / 2f, -thickness / 2f), 0f, 1f, 0f, null, width + thickness * 2f, thickness, ObjectType.Box));
            _screenWalls.Add(new PhysicsObject(new Vector2D(width / 2f, height + thickness / 2f), 0f, 1f, 0f, null, width + thickness * 2f, thickness, ObjectType.Box));
            _wallScreenWidth = width;
            _wallScreenHeight = height;
        }




        // Korrigerar överlappning och beräknar kollisionsimpulser med studs och friktion.
        private void ResolveCollision(PhysicsObject a, PhysicsObject b, Vector2D normal, float penetration, Vector2D contactPoint)
        {
            float totalInverseMass = a.InverseMass + b.InverseMass;

            if (totalInverseMass <= 0)
                return;

            if (Vector2D.Dot(b.Position - a.Position, normal) < 0f)
                normal *= -1f;

            // Fördelar positionskorrigeringen enligt kropparnas inversa massor.
            Vector2D correction = normal * (penetration / totalInverseMass);
            if (!a.IsStatic) a.Position -= correction * a.InverseMass;
            if (!b.IsStatic) b.Position += correction * b.InverseMass;

            // Kontaktpunktens hastighet inkluderar både rörelse och rotation.
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

            // Dämpar studs vid låga hastigheter för att undvika små, oönskade studs.
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

            // Friktionen verkar längs tangenten och begränsas av normalimpulsens storlek.
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

        // Beräknar 2D-kryssprodukten som ett skalärt värde.
        private static float Cross(Vector2D a, Vector2D b) => a.X * b.Y - a.Y * b.X;

        // Beräknar kroppens hastighet vid en punkt med hänsyn till rotation.
        private static Vector2D GetVelocityAtPoint(PhysicsObject body, Vector2D offset)
        {
            return body.Velocity + new Vector2D(-body.AngularVelocity * offset.Y, body.AngularVelocity * offset.X);
        }

        // Applicerar en impuls på både kroppens linjära och roterande rörelse.
        private static void ApplyCollisionImpulse(PhysicsObject body, float direction, Vector2D offset, Vector2D impulse)
        {
            if (body.IsStatic)
                return;

            body.Velocity += impulse * (direction * body.InverseMass);
            body.AngularVelocity += direction * Cross(offset, impulse) * body.InverseInertia;
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

        // Skapar punkter för respektive form och testar om kropparna överlappar.

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


        // Använder separerande axlar för att hitta överlappning, minsta penetration och normal.
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
                float radiusA = a.HalfWidth;
                float radiusB = b.HalfWidth;

                Vector2D direction = b.Position - a.Position;
                float distance = direction.Length();

                float totalRadius = radiusA + radiusB;

                if (distance >= totalRadius)
                    return false;

                if (distance < 0.0001f)
                    normal = new Vector2D(1f, 0f);
                else
                    normal = direction / distance;

                penetration = totalRadius - distance;

                contactPoint = a.Position +
                    normal * (radiusA - penetration * 0.5f);

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

                float radius = circle.HalfWidth;

                float cos = MathF.Cos(box.Rotation);
                float sin = MathF.Sin(box.Rotation);

                // Cirkelns centrum i boxens lokala koordinater.
                Vector2D relative = circle.Position - box.Position;

                float localX = relative.X * cos + relative.Y * sin;
                float localY = -relative.X * sin + relative.Y * cos;

                // Närmaste punkt på boxen.
                float closestX = Math.Clamp(
                    localX, -box.HalfWidth, box.HalfWidth);

                float closestY = Math.Clamp(
                    localY, -box.HalfHeight, box.HalfHeight);

                // Omvandla närmaste punkten till världens koordinater.
                Vector2D closestPoint = box.Position + new Vector2D(
                    closestX * cos - closestY * sin,
                    closestX * sin + closestY * cos);

                Vector2D difference = closestPoint - circle.Position;
                float distance = difference.Length();

                Vector2D circleToBox;

                if (distance > 0.0001f)
                {
                    // Cirkelns centrum ligger utanför boxen.
                    if (distance >= radius)
                        return false;

                    circleToBox = difference / distance;
                    penetration = radius - distance;
                }
                else
                {
                    // Cirkelns centrum ligger inuti boxen.
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

                        penetration = radius + distanceX;
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

                        penetration = radius + distanceY;
                    }

                    // Riktningen från cirkeln mot boxen är motsatt
                    // boxens utåtriktade normal när cirkeln är inuti.
                    circleToBox = outward * -1f;
                }

                // Normalen ska alltid peka från A mot B.
                normal = a.objectType == ObjectType.Ball
                    ? circleToBox
                    : circleToBox * -1f;

                contactPoint = closestPoint;

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

        private void GetArcCircle(
    Vector2D AA,
    Vector2D BA,
    float k,
    out Vector2D center,
    out float radius)
        {
            Vector2D v = BA - AA;

            float diameter = v.Length();

            if (diameter < 0.0001f)
            {
                center = Vector2D.Zero;
                radius = 0f;
                return;
            }

            Vector2D midpoint = (AA + BA) / 2f;

            float height = k * diameter;

            radius = (diameter * diameter) / (8f * height) + height / 2f;

            Vector2D normal = new Vector2D(-v.Y, v.X) / diameter;

            center = midpoint + normal * (radius - height);
        }
        private bool IsOnMinorArc(
    Vector2D P,
    Vector2D A,
    Vector2D B,
    Vector2D C)
        {
            Vector2D vA = A - C;
            Vector2D vB = B - C;
            Vector2D vP = P - C;

            float angleAB = MathF.Atan2(
                Cross(vA, vB),
                Vector2D.Dot(vA, vB));

            float angleAP = MathF.Atan2(
                Cross(vA, vP),
                Vector2D.Dot(vA, vP));

            const float epsilon = 0.0001f;

            if (angleAB >= 0f)
                return angleAP >= -epsilon &&
                       angleAP <= angleAB + epsilon;

            return angleAP <= epsilon &&
                   angleAP >= angleAB - epsilon;
        }

        private bool TryGetArcIntersection(
    Vector2D A1, Vector2D B1,
    Vector2D C1, float r1,
    Vector2D A2, Vector2D B2,
    Vector2D C2, float r2,
    out Vector2D intersection)
        {
            intersection = Vector2D.Zero;

            Vector2D delta = C2 - C1;
            float distance = delta.Length();

            if (distance < 0.0001f)
                return false;

            // Kontrollera om cirklarna överhuvudtaget skär varandra.
            if (distance > r1 + r2 ||
                distance < MathF.Abs(r1 - r2))
                return false;

            // Avstånd från C1 till skärningspunkternas mittlinje.
            float a = (r1 * r1 - r2 * r2 + distance * distance)
                      / (2f * distance);

            float hSquared = r1 * r1 - a * a;

            if (hSquared < -0.0001f)
                return false;

            float h = MathF.Sqrt(MathF.Max(0f, hSquared));

            Vector2D direction = delta / distance;

            Vector2D midpoint = C1 + direction * a;

            Vector2D perpendicular =
                new Vector2D(-direction.Y, direction.X);

            Vector2D p1 = midpoint + perpendicular * h;
            Vector2D p2 = midpoint - perpendicular * h;

            // Kontrollera om någon skärningspunkt ligger på båda bågarna.
            if (IsOnMinorArc(p1, A1, B1, C1) &&
                IsOnMinorArc(p1, A2, B2, C2))
            {
                intersection = p1;
                return true;
            }

            if (IsOnMinorArc(p2, A1, B1, C1) &&
                IsOnMinorArc(p2, A2, B2, C2))
            {
                intersection = p2;
                return true;
            }

            return false;
        }

        // Väljer ett ungefärligt kontaktläge mellan kropparnas stödytor.
        private Vector2D GetContactPoint(Vector2D[] pointsA, Vector2D[] pointsB, Vector2D normal)
        {
            Vector2D tangent = new Vector2D(-normal.Y, normal.X);
            GetSupportFeature(pointsA, normal, tangent, out float minTangentA, out float maxTangentA, out float normalA);
            GetSupportFeature(pointsB, normal * -1f, tangent, out float minTangentB, out float maxTangentB, out float normalB);

            float overlapMin = MathF.Max(minTangentA, minTangentB);
            float overlapMax = MathF.Min(maxTangentA, maxTangentB);
            float tangentPosition = overlapMin <= overlapMax
                ? (overlapMin + overlapMax) * 0.5f
                : (minTangentA + maxTangentA + minTangentB + maxTangentB) * 0.25f;

            return tangent * tangentPosition + normal * ((normalA + normalB) * 0.5f);
        }

        // Hämtar stödytans ytterpunkter längs tangenten och dess läge längs normalen.
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
                bestValue = MathF.Max(bestValue, Vector2D.Dot(points[i], supportDirection));

            minTangent = float.MaxValue;
            maxTangent = float.MinValue;
            normalPosition = 0f;
            int supportCount = 0;

            for (int i = 0; i < points.Length; i++)
            {
                if (bestValue - Vector2D.Dot(points[i], supportDirection) > 0.001f)
                    continue;

                float tangentProjection = Vector2D.Dot(points[i], tangent);
                minTangent = MathF.Min(minTangent, tangentProjection);
                maxTangent = MathF.Max(maxTangent, tangentProjection);
                normalPosition += Vector2D.Dot(points[i], normal);
                supportCount++;
            }

            normalPosition /= supportCount;
        }





        // Projicerar polygonens punkter på en axel och hittar min- och maxvärdet.
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

            return new Vector2D[]
            {
        point1,
        point2
            };
        }

        // Beräknar den roterade boxens fyra hörn i världens koordinater.
        private Vector2D[] GetBoxCorners(PhysicsObject body)
        {
            float rotation = body.Rotation;

            float cos = MathF.Cos(rotation);
            float sin = MathF.Sin(rotation);

            float width = body.HalfWidth * 2f;
            float height = body.HalfHeight * 2f;

            Vector2D corner1 = new Vector2D(body.Position.X + (-width / 2f * cos - -height / 2f * sin), body.Position.Y + (-width / 2f * sin + -height / 2f * cos));

            Vector2D corner2 = new Vector2D(body.Position.X + (width / 2f * cos - -height / 2f * sin), body.Position.Y + (width / 2f * sin + -height / 2f * cos));

            Vector2D corner3 = new Vector2D(body.Position.X + (width / 2f * cos - height / 2f * sin), body.Position.Y + (width / 2f * sin + height / 2f * cos));

            Vector2D corner4 = new Vector2D(body.Position.X + (-width / 2f * cos - height / 2f * sin), body.Position.Y + (-width / 2f * sin + height / 2f * cos));

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