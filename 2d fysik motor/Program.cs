using System;
using _2d_fysik_motor;
using Raylib_cs;

namespace _2d_fysik_motor;

internal static class Program
{
    [System.STAThread]
    public static void Main()
    {
        int screenWidth = 1910;
        int screenHeight = 900;
        Raylib.InitWindow(screenWidth, screenHeight, "Fysikmotor - Skapa objekt med musen");
        Raylib.SetTargetFPS(1000);

        // Skapa fysikvärlden
        WorldPhysics world = new WorldPhysics();
        Random rng = new Random();

        while (!Raylib.WindowShouldClose())
        {
            float deltaTime = Raylib.GetFrameTime();

            // --- INPUT: Skapa objekt vid vänsterklick ---
            if (Raylib.IsMouseButtonPressed(MouseButton.Left))
            {
                // Hämta muspositionen
                float mouseX = Raylib.GetMouseX();
                float mouseY = Raylib.GetMouseY();
                //for (int i = 1; i < 10; i++)
                {
                    //for(int x=0;x<10;x++)
                    {
                        // Skapa objektet i motorn
                        PhysicsObject newBall = world.AddBody(new Vector2D(mouseX, mouseY), mass: 1.5f, friction: 0.4f, radius: 15f, ObjectType.Box);
                        newBall.Restitution = 0.75f; // Studsighet

                        // Ge den en liten slumpmässig knuff i X-led (borde ändras till 0)
                        newBall.Velocity = new Vector2D(0, 0);
                    }
                }
            }

            // --- FYSISK STEG ---
            world.Step(deltaTime, screenWidth, screenHeight);

            // --- RITA (VISUALISERING) ---
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