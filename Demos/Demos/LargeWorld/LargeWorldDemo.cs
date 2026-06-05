using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Numerics;

namespace Demos.Demos.LargeWorld;

/// <summary>
/// Demonstrates a large world made from many adjacent 10 by 10 meter simulations.
/// </summary>
public class LargeWorldDemo : Demo
{
    LargeWorldGrid grid;
    RegionCoordinate displayOrigin;
    Sphere renderedBodyShape;
    Box renderedRegionFloor;
    Box renderedRegionBorder;

    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(0, 16, 28);
        camera.Yaw = 0;
        camera.Pitch = -0.45f;

        //The harness expects a primary Simulation. This demo renders the distributed simulations manually,
        //so the primary simulation is just a tiny placeholder for timing and grabber plumbing.
        Simulation = Simulation.Create(
            BufferPool,
            new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)),
            new DemoPoseIntegratorCallbacks(Vector3.Zero),
            new SolveDescription(1, 1));

        grid = new LargeWorldGrid(new Vector3(0, -10, 0));
        displayOrigin = new RegionCoordinate(0, 0);
        renderedBodyShape = new Sphere(0.35f);
        renderedRegionFloor = new Box(LargeWorldGrid.DefaultRegionSize - 0.08f, 0.04f, LargeWorldGrid.DefaultRegionSize - 0.08f);
        renderedRegionBorder = new Box(LargeWorldGrid.DefaultRegionSize, 0.08f, 0.08f);

        AddRunner(new RegionCoordinate(0, 0), new Vector3(4.4f, 1.2f, -3.5f), new Vector3(3.8f, 0, 1.5f));
        AddRunner(new RegionCoordinate(0, 0), new Vector3(-4.4f, 1.2f, 3.5f), new Vector3(-3.4f, 0, -1.2f));
        AddRunner(new RegionCoordinate(0, 0), new Vector3(-1.5f, 1.2f, 4.4f), new Vector3(1.4f, 0, 3.6f));
        AddRunner(new RegionCoordinate(0, 0), new Vector3(1.5f, 1.2f, -4.4f), new Vector3(-1.2f, 0, -3.2f));
        AddRunner(new RegionCoordinate(1, 0), new Vector3(-4.6f, 1.2f, 0), new Vector3(-2.8f, 0, 2.2f));
        AddRunner(new RegionCoordinate(-1, 0), new Vector3(4.6f, 1.2f, 0), new Vector3(2.6f, 0, -2.4f));
    }

    void AddRunner(RegionCoordinate region, Vector3 localPosition, Vector3 linearVelocity)
    {
        grid.AddSphere(region, localPosition, linearVelocity);
    }

    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        grid.Timestep(TimestepDuration, ThreadDispatcher);
        Simulation.Timestep(TimestepDuration, ThreadDispatcher);
    }

    public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
    {
        DrawVisibleRegions(renderer);
        DrawBodies(renderer);

        var bottomY = renderer.Surface.Resolution.Y;
        renderer.TextBatcher.Write(text.Clear().Append("World coordinates are long region indices plus local float poses; each region is a 10m simulation."), new Vector2(16, bottomY - 64), 16, Vector3.One, font);
        renderer.TextBatcher.Write(text.Clear().Append("Crossing an edge removes the body from one simulation and creates it in the adjacent region with a recentered pose."), new Vector2(16, bottomY - 48), 16, Vector3.One, font);
        renderer.TextBatcher.Write(text.Clear().Append("The transfer record is the same kind of payload a region server would send to a neighboring server."), new Vector2(16, bottomY - 32), 16, Vector3.One, font);
        renderer.TextBatcher.Write(text.Clear().Append("Active regions: ").Append(grid.RegionCount).Append(" Bodies: ").Append(grid.BodyCount).Append(" Transfers: ").Append(grid.TransferCount), new Vector2(16, bottomY - 16), 16, Vector3.One, font);
    }

    void DrawVisibleRegions(Renderer renderer)
    {
        const int radius = 2;
        for (int z = -radius; z <= radius; ++z)
        {
            for (int x = -radius; x <= radius; ++x)
            {
                var coordinate = new RegionCoordinate(displayOrigin.X + x, displayOrigin.Z + z);
                var center = new Vector3(x * grid.RegionSize, -0.06f, z * grid.RegionSize);
                var occupied = grid.GetBodyCount(coordinate) > 0;
                var color = occupied ? new Vector3(0.15f, 0.28f, 0.42f) : new Vector3(0.12f, 0.12f, 0.12f);
                renderer.Shapes.AddShape(renderedRegionFloor, Simulation.Shapes, new RigidPose(center, Quaternion.Identity), color);

                var borderColor = coordinate == displayOrigin ? new Vector3(0.7f, 0.7f, 0.9f) : new Vector3(0.35f, 0.35f, 0.35f);
                var northSouth = new RigidPose(center + new Vector3(0, 0.04f, grid.RegionSize * 0.5f), Quaternion.Identity);
                var eastWest = new RigidPose(center + new Vector3(grid.RegionSize * 0.5f, 0.04f, 0), QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, MathF.PI * 0.5f));
                renderer.Shapes.AddShape(renderedRegionBorder, Simulation.Shapes, northSouth, borderColor);
                renderer.Shapes.AddShape(renderedRegionBorder, Simulation.Shapes, eastWest, borderColor);
            }
        }
    }

    void DrawBodies(Renderer renderer)
    {
        foreach (var state in grid.EnumerateBodyStates())
        {
            var displayPose = state.LocalPose;
            displayPose.Position = grid.GetDisplayPosition(state, displayOrigin);
            renderer.Shapes.AddShape(renderedBodyShape, Simulation.Shapes, displayPose, GetBodyColor(state.Id));
        }
    }

    static Vector3 GetBodyColor(int bodyId)
    {
        return (bodyId % 6) switch
        {
            0 => new Vector3(0.95f, 0.38f, 0.25f),
            1 => new Vector3(0.25f, 0.75f, 0.95f),
            2 => new Vector3(0.35f, 0.9f, 0.4f),
            3 => new Vector3(0.95f, 0.78f, 0.25f),
            4 => new Vector3(0.8f, 0.45f, 0.95f),
            _ => new Vector3(0.95f, 0.95f, 0.95f),
        };
    }

    protected override void OnDispose()
    {
        grid?.Dispose();
    }
}
