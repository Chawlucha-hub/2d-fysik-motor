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
                    // Skapa objektet i motorn
                    PhysicsObject newBall = world.AddBody(new Vector2D(mouseX, mouseY), mass: 1.5f, friction: 0.4f , 20f, ObjectType.Ball);
                    newBall.Restitution = 0.75f; // Studsighet

                    // Ge den en liten slumpmässig knuff i X-led (borde ändras till 0)
                    newBall.Velocity = new Vector2D(0, 0);
                }
            }

            // --- FYSISK STEG ---
            world.Step(deltaTime, screenWidth, screenHeight);

            // --- RITA (VISUALISERING) ---
            Raylib.BeginDrawing();
            Raylib.ClearBackground(Color.RayWhite);

            Raylib.DrawText("Vänsterklicka för att skapa nya fysikobjekt!", 12, 12, 20, Color.DarkGray);
            Raylib.DrawText($"Antal objekt: {world.Bodies.Count}", 12, 35, 18, Color.Maroon);

            // Loopa igenom motorns alla objekt och rita dem på skärmen
            foreach (var body in world.Bodies)
            {
                int bodyPositionX = (int)body.Position.X;
                int bodyPositionY = (int)body.Position.Y;
                switch (body.objectType) { 
                    case ObjectType.Ball:
                        Raylib.DrawCircle(bodyPositionX, bodyPositionY, 20f, Color.DarkPurple);
                        break;
                    case ObjectType.Box:
                        Raylib.DrawRectangle(bodyPositionX, bodyPositionY, 20, 20, Color.DarkPurple);
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

            Raylib.EndDrawing();
        }

        Raylib.CloseWindow();
    }
}