using Raylib_cs;

namespace HelloWorld;

internal static class Program
{
    // STAThread is required if you deploy using NativeAOT on Windows
    // See https://github.com/raylib-cs/raylib-cs/issues/301
    [System.STAThread]
    public static void Main()
    {
        int x = 800;
        int y = 480;
        Raylib.InitWindow(x, y, "Hello, World");

        while (!Raylib.WindowShouldClose())
        {
            Raylib.BeginDrawing();
            Raylib.ClearBackground(Color.White);

            Raylib.DrawText("Hello, world!", 12, 12, 20, Color.Black);
            Raylib.DrawCircle(x/2, y/2, 80, Color.DarkPurple);

            Raylib.EndDrawing();
        }

        Raylib.CloseWindow();
    }
}