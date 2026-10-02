"""The one place `arris` touches numpy, and only when asked."""


def mesh_to_numpy(mesh):
    """`(positions, triangles)` of `mesh` as `(n, 3)` float64 and uint32
    arrays, copied out of the mesh's bytes."""
    try:
        import numpy
    except ImportError as error:
        raise ImportError(
            "Mesh.to_numpy() needs numpy, which arris does not require: "
            "install it with `pip install numpy`, or read the mesh's "
            "`positions` and `triangles` bytes (little-endian float64 and "
            "uint32) yourself"
        ) from error
    positions = numpy.frombuffer(mesh.positions, dtype="<f8").reshape(-1, 3)
    triangles = numpy.frombuffer(mesh.triangles, dtype="<u4").reshape(-1, 3)
    return (
        positions.astype(numpy.float64, copy=True),
        triangles.astype(numpy.uint32, copy=True),
    )
