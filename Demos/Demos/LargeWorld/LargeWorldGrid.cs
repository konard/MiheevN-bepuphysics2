using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Numerics;

namespace Demos.Demos.LargeWorld;

public readonly struct RegionCoordinate : IEquatable<RegionCoordinate>
{
    public readonly long X;
    public readonly long Z;

    public RegionCoordinate(long x, long z)
    {
        X = x;
        Z = z;
    }

    public bool Equals(RegionCoordinate other)
    {
        return X == other.X && Z == other.Z;
    }

    public override bool Equals(object obj)
    {
        return obj is RegionCoordinate coordinate && Equals(coordinate);
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(X, Z);
    }

    public override string ToString()
    {
        return $"({X}, {Z})";
    }

    public static bool operator ==(RegionCoordinate a, RegionCoordinate b)
    {
        return a.Equals(b);
    }

    public static bool operator !=(RegionCoordinate a, RegionCoordinate b)
    {
        return !a.Equals(b);
    }
}

public readonly struct LargeWorldBodyState
{
    public readonly int Id;
    public readonly RegionCoordinate Region;
    public readonly RigidPose LocalPose;
    public readonly BodyVelocity Velocity;

    public LargeWorldBodyState(int id, RegionCoordinate region, RigidPose localPose, BodyVelocity velocity)
    {
        Id = id;
        Region = region;
        LocalPose = localPose;
        Velocity = velocity;
    }
}

public sealed class LargeWorldGrid : IDisposable
{
    public const float DefaultRegionSize = 10f;

    public readonly float RegionSize;
    public readonly Vector3 Gravity;

    readonly Dictionary<RegionCoordinate, Region> regions = new();
    readonly Dictionary<int, BodyRecord> bodies = new();
    readonly List<PendingTransfer> pendingTransfers = new();
    int nextBodyId;
    int transferCount;
    bool disposed;

    public LargeWorldGrid(Vector3 gravity, float regionSize = DefaultRegionSize)
    {
        if (regionSize <= 0)
            throw new ArgumentOutOfRangeException(nameof(regionSize));

        Gravity = gravity;
        RegionSize = regionSize;
    }

    public int RegionCount
    {
        get { return regions.Count; }
    }

    public int BodyCount
    {
        get { return bodies.Count; }
    }

    public int TransferCount
    {
        get { return transferCount; }
    }

    public static RegionCoordinate FindContainingRegion(RegionCoordinate currentRegion, Vector3 localPosition, float regionSize, out Vector3 localPositionInRegion)
    {
        if (regionSize <= 0)
            throw new ArgumentOutOfRangeException(nameof(regionSize));

        var xOffset = GetRegionOffset(localPosition.X, regionSize);
        var zOffset = GetRegionOffset(localPosition.Z, regionSize);
        localPositionInRegion = new Vector3(
            localPosition.X - xOffset * regionSize,
            localPosition.Y,
            localPosition.Z - zOffset * regionSize);
        return new RegionCoordinate(currentRegion.X + xOffset, currentRegion.Z + zOffset);
    }

    public int AddSphere(RegionCoordinate regionCoordinate, Vector3 localPosition, Vector3 linearVelocity)
    {
        var region = GetOrCreateRegion(regionCoordinate);
        var description = BodyDescription.CreateDynamic(
            localPosition,
            linearVelocity,
            region.SphereInertia,
            region.SphereShape,
            0.01f);
        var handle = region.Simulation.Bodies.Add(description);
        var id = nextBodyId++;
        region.BodyHandlesById.Add(id, handle);
        bodies.Add(id, new BodyRecord(id, regionCoordinate, handle));
        return id;
    }

    public LargeWorldBodyState GetBodyState(int id)
    {
        if (!bodies.TryGetValue(id, out var body))
            throw new ArgumentOutOfRangeException(nameof(id));

        var region = regions[body.Region];
        region.Simulation.Bodies.GetDescription(body.Handle, out var description);
        return new LargeWorldBodyState(id, body.Region, description.Pose, description.Velocity);
    }

    public int GetBodyCount(RegionCoordinate regionCoordinate)
    {
        return regions.TryGetValue(regionCoordinate, out var region) ? region.BodyHandlesById.Count : 0;
    }

    public IEnumerable<LargeWorldBodyState> EnumerateBodyStates()
    {
        foreach (var body in bodies.Values)
        {
            var region = regions[body.Region];
            region.Simulation.Bodies.GetDescription(body.Handle, out var description);
            yield return new LargeWorldBodyState(body.Id, body.Region, description.Pose, description.Velocity);
        }
    }

    public Vector3 GetDisplayPosition(in LargeWorldBodyState state, RegionCoordinate displayOrigin)
    {
        var regionOffset = new Vector3(
            (float)(state.Region.X - displayOrigin.X) * RegionSize,
            0,
            (float)(state.Region.Z - displayOrigin.Z) * RegionSize);
        return regionOffset + state.LocalPose.Position;
    }

    public void Timestep(float dt, IThreadDispatcher threadDispatcher = null)
    {
        foreach (var region in regions.Values)
        {
            region.Simulation.Timestep(dt, threadDispatcher);
        }

        QueueTransfers();
        ApplyTransfers();
    }

    Region GetOrCreateRegion(RegionCoordinate coordinate)
    {
        if (regions.TryGetValue(coordinate, out var region))
            return region;

        region = new Region(coordinate, RegionSize, Gravity);
        regions.Add(coordinate, region);
        return region;
    }

    void QueueTransfers()
    {
        pendingTransfers.Clear();
        foreach (var body in bodies.Values)
        {
            var region = regions[body.Region];
            region.Simulation.Bodies.GetDescription(body.Handle, out var description);
            var containingRegion = FindContainingRegion(body.Region, description.Pose.Position, RegionSize, out var positionInRegion);
            if (containingRegion != body.Region)
            {
                description.Pose.Position = positionInRegion;
                pendingTransfers.Add(new PendingTransfer(body.Id, body.Region, containingRegion, description));
            }
        }
    }

    void ApplyTransfers()
    {
        for (int i = 0; i < pendingTransfers.Count; ++i)
        {
            var transfer = pendingTransfers[i];
            var body = bodies[transfer.BodyId];
            var sourceRegion = regions[transfer.SourceRegion];
            var destinationRegion = GetOrCreateRegion(transfer.DestinationRegion);

            sourceRegion.Simulation.Bodies.Remove(body.Handle);
            sourceRegion.BodyHandlesById.Remove(body.Id);

            //This description is the payload a networked deployment would hand to the neighboring region server.
            var description = transfer.Description;
            description.Collidable.Shape = destinationRegion.SphereShape.Shape;

            var destinationHandle = destinationRegion.Simulation.Bodies.Add(description);
            destinationRegion.BodyHandlesById.Add(body.Id, destinationHandle);

            body.Region = transfer.DestinationRegion;
            body.Handle = destinationHandle;
            ++transferCount;
        }
    }

    static long GetRegionOffset(float localCoordinate, float regionSize)
    {
        var halfSize = 0.5 * regionSize;
        return (long)Math.Floor((localCoordinate + halfSize) / regionSize);
    }

    public void Dispose()
    {
        if (disposed)
            return;

        disposed = true;
        foreach (var region in regions.Values)
        {
            region.Dispose();
        }
    }

    sealed class BodyRecord
    {
        public readonly int Id;
        public RegionCoordinate Region;
        public BodyHandle Handle;

        public BodyRecord(int id, RegionCoordinate region, BodyHandle handle)
        {
            Id = id;
            Region = region;
            Handle = handle;
        }
    }

    readonly struct PendingTransfer
    {
        public readonly int BodyId;
        public readonly RegionCoordinate SourceRegion;
        public readonly RegionCoordinate DestinationRegion;
        public readonly BodyDescription Description;

        public PendingTransfer(int bodyId, RegionCoordinate sourceRegion, RegionCoordinate destinationRegion, BodyDescription description)
        {
            BodyId = bodyId;
            SourceRegion = sourceRegion;
            DestinationRegion = destinationRegion;
            Description = description;
        }
    }

    sealed class Region : IDisposable
    {
        public readonly RegionCoordinate Coordinate;
        public readonly BufferPool BufferPool;
        public readonly Simulation Simulation;
        public readonly CollidableDescription SphereShape;
        public readonly BodyInertia SphereInertia;
        public readonly Dictionary<int, BodyHandle> BodyHandlesById = new();

        public Region(RegionCoordinate coordinate, float regionSize, Vector3 gravity)
        {
            Coordinate = coordinate;
            BufferPool = new BufferPool();
            Simulation = Simulation.Create(
                BufferPool,
                new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)),
                new DemoPoseIntegratorCallbacks(gravity, linearDamping: 0, angularDamping: 0),
                new SolveDescription(8, 1));

            var sphere = new Sphere(0.35f);
            SphereShape = Simulation.Shapes.Add(sphere);
            SphereInertia = sphere.ComputeInertia(1);

            var floorShape = Simulation.Shapes.Add(new Box(regionSize, 0.1f, regionSize));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.05f, 0), floorShape));
        }

        public void Dispose()
        {
            Simulation.Dispose();
            BufferPool.Clear();
        }
    }
}
