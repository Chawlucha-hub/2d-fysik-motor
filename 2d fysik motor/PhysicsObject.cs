using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace _2d_fysik_motor
{
    public struct Vector2D
    {
        public float X;
        public float Y;

        public Vector2D(float x, float y)
        {
            X = x;
            Y = y;
        }

        // fysik operationen
        public static Vector2D operator +(Vector2D a, Vector2D b) => new Vector2D(a.X + b.X, a.Y + b.Y);
        public static Vector2D operator -(Vector2D a, Vector2D b) => new Vector2D(a.X - b.X, a.Y - b.Y);
        public static Vector2D operator *(Vector2D a, float scalar) => new Vector2D(a.X * scalar, a.Y * scalar);
        public static Vector2D operator /(Vector2D a, float scalar) { return new Vector2D(a.X / scalar, a.Y / scalar); }


        public float Length()
        {
            return MathF.Sqrt(X * X + Y * Y);
        }
        public Vector2D Normalized()
        {
            float length = Length();

            if (length == 0)
            {
                return Zero;
            }
            return this / length;
        }
        public static float Dot(Vector2D a, Vector2D b)
        {
            return a.X * b.X + a.Y * b.Y;
        }
        public static Vector2D Zero => new Vector2D(0, 0);
    }




    public class PhysicsObject
    {
        // egenskaper
        public Vector2D Position;
        public Vector2D Velocity;
        public Vector2D ForceAccumulator;

        public float Mass { get; private set; }
        public float InverseMass { get; private set; } // Används för prestanda (1 / Mass)
        public float Restitution { get; set; } = 0.8f; // Bounciness (0 = inga studsar, 1 = perfekt studs)
        public bool IsStatic => InverseMass == 0f;     // Om objektet är obevägligt (t.ex. ett golv)

        public PhysicsObject(Vector2D position , float mass)
        {
            // gör väderna anvendbara
            Position = position;
            SetMass(mass);
            Velocity = Vector2D.Zero;
            ForceAccumulator = Vector2D.Zero;
        }
        public void SetMass(float mass)
        {
            Mass = mass;
            // Om massan är 0 räknas det som ett statiskt objekt med oändlig massa
            InverseMass = (mass > 0f) ? 1f / mass : 0f;
        }
        public void AddForce(Vector2D force)
        {
            if (IsStatic) return;
            ForceAccumulator += force;
        }
        public void Update(float deltaTime)
        {
            if (IsStatic) return;

            // F = m * a  =>  a = F / m  =>  a = F * InvMass
            Vector2D acceleration = ForceAccumulator * InverseMass;

            // Integrera hastighet: v = v + a * dt
            Velocity += acceleration * deltaTime;

            // Integrera position: p = p + v * dt
            Position += Velocity * deltaTime;

            // Nollställ krafter inför nästa bildruta
            ForceAccumulator = Vector2D.Zero;
        }


    }
}
