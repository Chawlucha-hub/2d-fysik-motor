using _2d_fysik_motor;
using Raylib_cs;
using System;
using System.Threading;

namespace _2d_fysik_motor;

// Startpunkt för fönstret och programmets huvudloop.
internal static class Program
{
    [System.STAThread]
    // Initierar programmet och kör fysiksimuleringen tills fönstret stängs.
    public static void Main()
    {
        int screenWidth = 1910;
        int screenHeight = 900;

        // Skapar Raylib-fönstret och väljer önskad bildfrekvens.
        Raylib.InitWindow(screenWidth, screenHeight, "Fysikmotor - Skapa objekt med musen");
        Raylib.SetTargetFPS(1000);

        // Skapar fysikvärlden och en kropp med massan noll, som därför är statisk.
        WorldPhysics world = new WorldPhysics();
        Random rng = new Random();

        PhysicsObject newGoyBox = world.AddBody(new Vector2D(screenWidth / 2, screenHeight / 2), 0f, 0f, 0f, 100f, 30f, ObjectType.Box);
        //PhysicsObject newGoyBox2 = world.AddBody(new Vector2D(screenWidth / 2, -screenHeight + 100), 0f, 0f, 0f, 10000f, 30f, ObjectType.Box);


        // Huvudloop: läser indata, uppdaterar fysiken och ritar scenen.
        while (!Raylib.WindowShouldClose())
        {
            float deltaTime = Raylib.GetFrameTime();
            float mouseX = Raylib.GetMouseX();
            float mouseY = Raylib.GetMouseY();

            // Tangent E skapar en box vid muspekaren.
            if (Raylib.IsKeyPressed(KeyboardKey.E))
            {
                //for (int i = 1; i < 10; i++)
                {
                    //for(int x=0;x<10;x++)
                    {
                        // Skapar boxen och anger dess studsförmåga och starthastighet.
                        PhysicsObject newBox = world.AddBody(new Vector2D(mouseX, mouseY), mass: 1.5f, friction: 0.4f, 20f, 20f, 20f, ObjectType.Box);
                        newBox.Restitution = 0.75f; // Studsighet

                        // Objektet börjar stilla.
                        newBox.Velocity = new Vector2D(0, 0);
                    }
                }
            }

            // Tangent Q skapar en bollformad kropp vid muspekaren.
            if (Raylib.IsKeyPressed(KeyboardKey.Q))
            {
                PhysicsObject newBall = world.AddBody(new Vector2D(mouseX, mouseY), mass: 1.5f, friction: 0.4f, 20f, 20f, 20f, ObjectType.Ball);
                newBall.Restitution = 0.75f; // Studsighet

                // Objektet börjar stilla.
                newBall.Velocity = new Vector2D(0, 0);
            }

            // Tangent G skapar en långsmal box vid muspekaren.
            if (Raylib.IsKeyPressed(KeyboardKey.G))
            {
                PhysicsObject newBox = world.AddBody(new Vector2D(mouseX, mouseY), mass: 1.5f, friction: 0.4f, 20f, 200f, 20f, ObjectType.Box);
                newBox.Restitution = 0.75f; // Studsighet

                // Objektet börjar stilla.
                newBox.Velocity = new Vector2D(0, 0);
            }

            // Uppdaterar krafter, rörelser och kollisioner för den här bildrutan.
            world.Step(deltaTime, screenWidth, screenHeight);

            // Ritar bakgrunden, räknaren och alla fysikkroppar.
            Raylib.BeginDrawing();
            Raylib.ClearBackground(Color.RayWhite);

            Raylib.DrawText("Vänsterklicka för att skapa nya fysikobjekt!", 12, 12, 20, Color.DarkGray);
            Raylib.DrawText($"Antal objekt: {world.Bodies.Count}", 12, 35, 18, Color.Maroon);

            world.DrawObjects();

            Raylib.EndDrawing();
        }

        Raylib.CloseWindow();
    }
}