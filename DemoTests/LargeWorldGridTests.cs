using System.Numerics;
using Demos;
using Demos.Demos.LargeWorld;
using Xunit;

namespace DemoTests;

public static class LargeWorldGridTests
{
    [Fact]
    public static void FindContainingRegionRecentersAcrossHugeRegionIndices()
    {
        var startingRegion = new RegionCoordinate(long.MaxValue / 3, long.MinValue / 3);

        var destination = LargeWorldGrid.FindContainingRegion(
            startingRegion,
            new Vector3(-25.25f, 1.25f, 16.75f),
            LargeWorldGrid.DefaultRegionSize,
            out var localPosition);

        Assert.Equal(new RegionCoordinate(startingRegion.X - 3, startingRegion.Z + 2), destination);
        Assert.Equal(4.75f, localPosition.X, 5);
        Assert.Equal(1.25f, localPosition.Y, 5);
        Assert.Equal(-3.25f, localPosition.Z, 5);
    }

    [Fact]
    public static void DynamicBodyTransfersToAdjacentRegion()
    {
        using var grid = new LargeWorldGrid(Vector3.Zero);
        var bodyId = grid.AddSphere(
            new RegionCoordinate(0, 0),
            new Vector3(4.95f, 2, 0),
            new Vector3(12, 0, 0));

        grid.Timestep(Demo.TimestepDuration);

        var state = grid.GetBodyState(bodyId);
        Assert.Equal(new RegionCoordinate(1, 0), state.Region);
        Assert.InRange(state.LocalPose.Position.X, -4.86f, -4.84f);
        Assert.Equal(2, state.LocalPose.Position.Y, 5);
        Assert.Equal(0, state.LocalPose.Position.Z, 5);
        Assert.Equal(12, state.Velocity.Linear.X, 5);
        Assert.Equal(0, grid.GetBodyCount(new RegionCoordinate(0, 0)));
        Assert.Equal(1, grid.GetBodyCount(new RegionCoordinate(1, 0)));
    }
}
